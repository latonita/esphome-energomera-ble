#include "energomera_ble.h"

#include "esphome/core/log.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include <esp_gatt_defs.h>
#include <esp_heap_caps.h>

#define SET_STATE(st) \
  { \
    ESP_LOGV(TAG, "State change request:"); \
    this->set_state_(st); \
  }

static constexpr uint32_t BOOT_TIMEOUT_MS = 10 * 1000;
static constexpr uint32_t SAFEGUARD_INTERVAL_MS = 30 * 1000;
namespace esphome {
namespace energomera_ble {

static const char *const TAG = "energomera_ble";

namespace espbt = esphome::esp32_ble_tracker;

static const espbt::ESPBTUUID ENERGOMERA_SERVICE_UUID =
    espbt::ESPBTUUID::from_raw("b91b0100-8bef-45e2-97c3-1cd862d914df");
static const espbt::ESPBTUUID ENERGOMERA_VER_UUID = espbt::ESPBTUUID::from_raw("b91b0101-8bef-45e2-97c3-1cd862d914df");
static const espbt::ESPBTUUID ENERGOMERA_TXN_UUID = espbt::ESPBTUUID::from_raw("b91b0105-8bef-45e2-97c3-1cd862d914df");
static const espbt::ESPBTUUID ENERGOMERA_RX_UUIDS[RX_HANDLES_NUM] = {
    espbt::ESPBTUUID::from_raw("b91b0106-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b0107-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b0108-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b0109-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b010a-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b010b-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b010c-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b010d-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b010e-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b010f-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b0110-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b0111-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b0112-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b0113-8bef-45e2-97c3-1cd862d914df"),
    espbt::ESPBTUUID::from_raw("b91b0114-8bef-45e2-97c3-1cd862d914df"),
};

static char empty_str[] = "";

static char format_hex_char(uint8_t v) { return v >= 10 ? 'A' + (v - 10) : '0' + v; }

static std::string format_frame_pretty(const uint8_t *data, size_t length) {
  if (length == 0)
    return "";
  std::string ret;
  ret.resize(3 * length - 1);
  std::ostringstream ss(ret);

  for (size_t i = 0; i < length; i++) {
    switch (data[i]) {
      case 0x00:
        ss << "<NUL>";
        break;
      case 0x01:
        ss << "<SOH>";
        break;
      case 0x02:
        ss << "<STX>";
        break;
      case 0x03:
        ss << "<ETX>";
        break;
      case 0x04:
        ss << "<EOT>";
        break;
      case 0x05:
        ss << "<ENQ>";
        break;
      case 0x06:
        ss << "<ACK>";
        break;
      case 0x0d:
        ss << "<CR>";
        break;
      case 0x0a:
        ss << "<LF>";
        break;
      case 0x15:
        ss << "<NAK>";
        break;
      case 0x20:
        ss << "<SP>";
        break;
      default:
        if (data[i] <= 0x20 || data[i] >= 0x7f) {
          ss << "<" << format_hex_char((data[i] & 0xF0) >> 4) << format_hex_char(data[i] & 0x0F) << ">";
        } else {
          ss << (char) data[i];
        }
        break;
    }
  }
  if (length > 4)
    ss << " (" << length << ")";
  return ss.str();
}

static uint8_t apply_even_parity(uint8_t value) {
  uint8_t bit_count = 0;
  for (uint8_t bit = 0; bit < 7; bit++) {
    bit_count += (value >> bit) & 0x01;
  }
  uint8_t with_msb = value | 0x80;
  if ((bit_count & 0x01) != 0) {
    return with_msb;
  }
  return with_msb & 0x7F;
}

void EnergomeraBleComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Energomera BLE component");

  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "BLE client parent not configured");
    this->mark_failed();
    return;
  }

  this->tx_message_remaining_.reserve(128);
  this->response_buffer_.reserve(256);

  this->request_iter = this->sensors_.begin();
  this->sensor_iter = this->sensors_.begin();

  if (this->parent_->get_remote_bda() != nullptr) {
    esp_ble_gattc_cache_clean(this->parent_->get_remote_bda());
  }

  this->set_timeout(BOOT_TIMEOUT_MS, [this]() {
    ESP_LOGD(TAG, "Boot timeout, component is ready to use");
    SET_STATE(FsmState::IDLE);
  });

  this->internal_safeguard_();
  ESP_LOGI(TAG, "Energomera BLE setup complete.");
}

const char *ble_client_state_to_string(espbt::ClientState state) {
  switch (state) {
    case espbt::ClientState::INIT:
      return "INIT";
    case espbt::ClientState::DISCONNECTING:
      return "DISCONNECTING";
    case espbt::ClientState::IDLE:
      return "IDLE";
    case espbt::ClientState::DISCOVERED:
      return "DISCOVERED";
    case espbt::ClientState::CONNECTING:
      return "CONNECTING";
    case espbt::ClientState::CONNECTED:
      return "CONNECTED";
    case espbt::ClientState::ESTABLISHED:
      return "ESTABLISHED";
    default:
      return "UNKNOWN";
  }
}

void EnergomeraBleComponent::internal_safeguard_() {
  static uint8_t error_states = 0;
  static uint8_t ble_connectings = 0;

  if (this->state_ == FsmState::ERROR) {
    error_states++;
  }

  auto my_ble_state = this->node_state;
  auto parent_ble_state = this->parent_->state();
  if (my_ble_state == espbt::ClientState::CONNECTING) {
    ble_connectings++;
  }

  ESP_LOGV(TAG, "Internal safeguard. FSM state is %s, My BLE state is %s, Parent BLE state is %s",
           this->state_to_string_(this->state_), ble_client_state_to_string(my_ble_state),
           ble_client_state_to_string(parent_ble_state));

  if (error_states > 10 || ble_connectings > 10) {
    ESP_LOGE(TAG, "Too many errors or BLE connecting states. Disconnecting.");
    delay(100);
    ble_connectings = 0;
    error_states = 0;
    this->parent_->disconnect();
    SET_STATE(FsmState::IDLE);
    // ESP.restart();
    // return;
  }

  this->set_timeout("energomera_internal_safeguard", SAFEGUARD_INTERVAL_MS, [this]() { this->internal_safeguard_(); });
}

void EnergomeraBleComponent::remove_bonding() {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "BLE client parent not configured");
    return;
  }
  if (this->parent_->get_remote_bda() != nullptr) {
    auto status = esp_ble_remove_bond_device(this->parent_->get_remote_bda());
    ESP_LOGI(TAG, "Bond removal. Status %d", status);
    esp_ble_gattc_cache_clean(this->parent_->get_remote_bda());
  }
  SET_STATE(FsmState::IDLE);
}

void EnergomeraBleComponent::register_sensor(EnergomeraBleSensorBase *sensor) {
  this->sensors_.insert({sensor->get_request(), sensor});
}

void EnergomeraBleComponent::loop() {
  ValueRefsArray vals;  // values from brackets, refs to this->buffers_.in
  char *in_param_ptr =
      (char *) &this->response_buffer_.data()[1];  // ref to second byte, first is STX/SOH in R1 requests

  switch (this->state_) {
    case FsmState::IDLE: {
    } break;

    case FsmState::START: {
    } break;

    case FsmState::RESOLVING: {
    } break;

    case FsmState::REQUESTING_FIRMWARE: {
    } break;

    case FsmState::WAITING_FIRMWARE: {
    } break;

    case FsmState::ENABLING_NOTIFICATION: {
    } break;

    case FsmState::WAITING_NOTIFICATION_ENABLE: {
    } break;

    case FsmState::PREPARING_COMMAND: {
      if (this->request_iter == this->sensors_.end()) {
        ESP_LOGI(TAG, "All requests sent");
        SET_STATE(FsmState::PUBLISH);
        this->parent_->disconnect();
        break;
      }
      this->response_buffer_.clear();
      auto req = this->request_iter->first;
      this->prepare_prog_frame_(req.c_str());
      ESP_LOGI(TAG, "Sending request %s (%u bytes payload)", req.c_str(),
               (unsigned) this->tx_message_remaining_.size());
      SET_STATE(FsmState::SENDING_COMMAND);

    } break;
    case FsmState::SENDING_COMMAND: {
      if (this->send_next_fragment_()) {
      } else {
        SET_STATE(FsmState::ERROR);
      }
    } break;

    case FsmState::WAITING_NOTIFICATION: {
    } break;

    case FsmState::READING_RESPONSE: {
      // reading packets
    } break;

    case FsmState::GOT_RESPONSE:
      // full packet received
      if (!this->response_buffer_.empty()) {
        uint8_t brackets_found = get_values_from_brackets_(in_param_ptr, vals);
        if (!brackets_found) {
          ESP_LOGE(TAG, "Invalid frame format: '%s'", in_param_ptr);
          return;
        }

        ESP_LOGD(TAG,
                 "Received name: '%s', values: %d, idx: 1(%s), 2(%s), 3(%s), 4(%s), 5(%s), 6(%s), 7(%s), 8(%s), 9(%s), "
                 "10(%s), 11(%s), 12(%s)",
                 in_param_ptr, brackets_found, vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7],
                 vals[8], vals[9], vals[10], vals[11]);

        if (in_param_ptr[0] == '\0') {
          if (vals[0][0] == 'E' && vals[0][1] == 'R' && vals[0][2] == 'R') {
            ESP_LOGE(TAG, "Request '%s' either not supported or malformed. Error code %s", in_param_ptr, vals[0]);
          } else {
            ESP_LOGE(TAG, "Request '%s' either not supported or malformed.", in_param_ptr);
          }
        } else {
          auto req = this->request_iter->first;

          if (this->request_iter->second->get_function() != in_param_ptr) {
            ESP_LOGE(TAG, "Returned data name mismatch. Skipping frame");
            return;
          }

          auto range = sensors_.equal_range(req);
          for (auto it = range.first; it != range.second; ++it) {
            if (!it->second->is_failed())
              set_sensor_value_(it->second, vals);
          }
        }
      }

      this->request_iter = this->sensors_.upper_bound(this->request_iter->first);
      this->response_buffer_.clear();
      SET_STATE(FsmState::PREPARING_COMMAND);
      break;

    case FsmState::PUBLISH: {
      if (this->sensor_iter != this->sensors_.end()) {
        this->sensor_iter->second->publish();
        this->sensor_iter++;
      } else {
        SET_STATE(FsmState::IDLE);
      }
    } break;

    case FsmState::ERROR:
      // do nothing
      break;
  }
}

uint8_t EnergomeraBleComponent::get_values_from_brackets_(char *line, ValueRefsArray &vals) {
  // line = "VOLTA(100.1)VOLTA(200.1)VOLTA(300.1)VOLTA(400.1)"
  vals.fill(empty_str);

  uint8_t idx = 0;
  bool got_param_name{false};
  char *p = line;
  while (*p && idx < VAL_NUM) {
    if (*p == '(') {
      if (!got_param_name) {
        got_param_name = true;
        *p = '\0';  // null-terminate param name
      }
      char *start = p + 1;
      char *end = strchr(start, ')');
      if (end) {
        *end = '\0';  // null-terminate value
        if (idx < VAL_NUM) {
          vals[idx++] = start;
        }
        p = end;
      }
    }
    p++;
  }
  return idx;  // at least one bracket found
}

bool char2float(const char *str, float &value) {
  char *end;
  value = strtof(str, &end);
  return *end == '\0' || *end == '*' || *end == '#';
}

// Get N-th value from comma-separated string, 1-based index
// line = "20.08.24,0.45991"
// get_nth_value_from_csv_(line, 1) -> "20.08.24"
// get_nth_value_from_csv_(line, 2) -> "0.45991"
char *EnergomeraBleComponent::get_nth_value_from_csv_(char *line, uint8_t idx) {
  if (idx == 0) {
    return line;
  }
  char *ptr;
  ptr = strtok(line, ",");
  while (ptr != nullptr) {
    if (idx-- == 1)
      return ptr;
    ptr = strtok(nullptr, ",");
  }
  return nullptr;
}

bool EnergomeraBleComponent::set_sensor_value_(EnergomeraBleSensorBase *sensor, ValueRefsArray &vals) {
  auto type = sensor->get_type();
  bool ret = true;

  uint8_t idx = sensor->get_index() - 1;
  if (idx >= VAL_NUM) {
    ESP_LOGE(TAG, "Invalid sensor index %u", idx);
    return false;
  }
  char str_buffer[128] = {'\0'};
  strncpy(str_buffer, vals[idx], 128);

  char *str = str_buffer;
  uint8_t sub_idx = sensor->get_sub_index();
  if (sub_idx == 0) {
    ESP_LOGV(TAG, "Setting value for sensor '%s', idx = %d to '%s'", sensor->get_request().c_str(), idx + 1, str);
  } else {
    ESP_LOGV(TAG, "Extracting value for sensor '%s', idx = %d, sub_idx = %d from '%s'", sensor->get_request().c_str(),
             idx + 1, sub_idx, str);
    str = this->get_nth_value_from_csv_(str, sub_idx);
    if (str == nullptr) {
      ESP_LOGE(TAG,
               "Cannot extract sensor value by sub-index %d. Is data comma-separated? Also note that sub-index starts "
               "from 1",
               sub_idx);
      str_buffer[0] = '\0';
      str = str_buffer;
    }
    ESP_LOGV(TAG, "Setting value using sub-index = %d, extracted sensor value is '%s'", sub_idx, str);
  }

#ifdef USE_SENSOR
  if (type == SensorType::SENSOR) {
    float f = 0;
    // todo: for non-energomeras... value can be "100.0" or "100.0*kWh" or "100.0#A"
    ret = str && str[0] && char2float(str, f);
    if (ret) {
      static_cast<EnergomeraBleSensor *>(sensor)->set_value(f);
    } else {
      ESP_LOGE(TAG, "Cannot convert incoming data to a number. Consider using a text sensor. Invalid data: '%s'", str);
    }
  }
#endif
#ifdef USE_TEXT_SENSOR
  if (type == SensorType::TEXT_SENSOR) {
    static_cast<EnergomeraBleTextSensor *>(sensor)->set_value(str);
  }
#endif
  return ret;
}

void EnergomeraBleComponent::try_connect() {
  if (this->state_ != FsmState::IDLE) {
    ESP_LOGW(TAG, "Not in IDLE state, can't start data collection. Current state is %s",
             this->state_to_string_(this->state_));
    return;
  }
  ESP_LOGI(TAG, "Initiating data collection from Energomera BLE device");
  this->request_iter = this->sensors_.begin();
  this->sensor_iter = this->sensors_.begin();

  auto ret = esp_ble_gatt_set_local_mtu(200);
  if (ret) {
    ESP_LOGE(TAG, "set local  MTU failed, error code = %x", ret);
  }

  /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;  // bonding with peer device after authentication
  esp_ble_io_cap_t iocap = ESP_IO_CAP_IN;  // NONE;                    // set the IO capability to No output No input
  uint8_t key_size = 16;                   // the key size should be 7~16 bytes
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t oob_support = ESP_BLE_OOB_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));

  // Enable bonding
  uint8_t bonding = 1;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &bonding, sizeof(uint8_t));

  this->parent_->connect();
}

void EnergomeraBleComponent::update() { this->try_connect(); }

void EnergomeraBleComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Energomera BLE Component");
  if (this->parent_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  target address: %s", this->parent_->address_str().c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  target address: not set");
  }
}

// void EnergomeraBleComponent::sync_address_from_parent_() {
//   if (this->address_set_ || this->parent_ == nullptr)
//     return;
//   uint8_t *remote = this->parent_->get_remote_bda();
//   if (remote == nullptr)
//     return;
//   bool nonzero = false;
//   for (uint8_t i = 0; i < 6; i++) {
//     if (remote[i] != 0) {
//       nonzero = true;
//       break;
//     }
//   }
//   if (!nonzero)
//     return;
//   std::memcpy(this->target_address_.data(), remote, 6);
//   this->address_set_ = true;
//   ESP_LOGD(TAG, "Using parent BLE address: %02X:%02X:%02X:%02X:%02X:%02X", remote[0], remote[1], remote[2],
//   remote[3],
//            remote[4], remote[5]);
// }

bool EnergomeraBleComponent::discover_characteristics_() {
  bool result{true};
  esphome::ble_client::BLECharacteristic *chr;
  ESP_LOGV(TAG, "Discovering Energomera characteristics...");
  if (!this->ch_handle_tx_) {
    chr = this->parent_->get_characteristic(ENERGOMERA_SERVICE_UUID, ENERGOMERA_TXN_UUID);
    if (chr == nullptr) {
      ESP_LOGW(TAG, "No TX/Notify found");
      result = false;
    } else {
      ESP_LOGD(TAG, "TX/Notify handle: 0x%04X", chr->handle);
      this->ch_handle_tx_ = chr->handle;
    }
  }
  auto *descr = this->parent_->get_config_descriptor(this->ch_handle_tx_);
  if (descr == nullptr) {
    ESP_LOGW(TAG, "No CCCD descriptor found");
    result = false;
  } else {
    ESP_LOGD(TAG, "CCCD descriptor handle: 0x%04X", descr->handle);
    this->ch_handle_cccd_ = descr->handle;
  }

  for (size_t i = 0; i < sizeof(this->ch_handles_rx_) / sizeof(this->ch_handles_rx_[0]); i++) {
    if (this->ch_handles_rx_[i] == 0) {
      chr = this->parent_->get_characteristic(ENERGOMERA_SERVICE_UUID, ENERGOMERA_RX_UUIDS[i]);
      if (chr == nullptr) {
        ESP_LOGW(TAG, "No RX%zu found", i);
        result = false;
      } else {
        ESP_LOGD(TAG, "RX %zu handle: 0x%04X", i, chr->handle);
        this->ch_handles_rx_[i] = chr->handle;
      }
    }
  }

  return result;
}

void EnergomeraBleComponent::prepare_prog_frame_(const std::string &request) {
  static uint8_t command_buffer[128];

  size_t len = snprintf((char *) command_buffer, sizeof(command_buffer), "/?!\x01R1\x02%s\x03", request.c_str());
  this->tx_message_remaining_.assign(command_buffer, command_buffer + len + 1);  // include null terminator

  int checksum = 0;
  for (size_t i = 0; i < tx_message_remaining_.size() - 5; ++i) {
    checksum += tx_message_remaining_[i + 4];
  }
  tx_message_remaining_[tx_message_remaining_.size() - 1] = static_cast<uint8_t>(checksum & 0x7F);

  for (auto &byte : this->tx_message_remaining_) {
    byte = apply_even_parity(byte & 0x7F);
  }
  this->tx_fragment_started_ = false;
  this->tx_sequence_counter_ = 0;
}

void EnergomeraBleComponent::prepare_request_(const std::string &request) { this->tx_message_remaining_.clear(); }

uint16_t EnergomeraBleComponent::get_max_payload_() const {
  if (this->mtu_ <= 4)
    return 0;
  return this->mtu_ - 4;
}

bool EnergomeraBleComponent::send_next_fragment_() {
  if (this->parent_ == nullptr || this->ch_handle_tx_ == 0)
    return false;

  if (this->tx_message_remaining_.empty())
    return false;

  uint16_t max_payload = this->get_max_payload_();
  if (max_payload == 0) {
    ESP_LOGW(TAG, "MTU too small to carry command data");
    return false;
  }

  bool more_after_this = this->tx_message_remaining_.size() > max_payload;
  uint16_t chunk_len = more_after_this ? max_payload : this->tx_message_remaining_.size();

  std::vector<uint8_t> packet(chunk_len + 1);
  if (!this->tx_fragment_started_) {
    this->tx_fragment_started_ = true;
    this->tx_sequence_counter_ = 0;
    if (more_after_this) {
      packet[0] = 0x00;
    } else {
      packet[0] = 0x80;
      if (this->mtu_ > 23)
        packet[0] |= 0x40;
    }
  } else {
    this->tx_sequence_counter_ = (this->tx_sequence_counter_ + 1) & 0x7F;
    packet[0] = this->tx_sequence_counter_;
    if (!more_after_this)
      packet[0] |= 0x80;
  }

  std::copy_n(this->tx_message_remaining_.begin(), chunk_len, packet.begin() + 1);

  esp_err_t status =
      esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->ch_handle_tx_,
                               packet.size(), packet.data(), ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "send_next_fragment_() esp_ble_gattc_write_char failed: %d", status);
    return false;
  }

  ESP_LOGV(TAG, "TX: %s", format_hex_pretty(packet.data(), packet.size()).c_str());

  this->tx_message_remaining_.erase(this->tx_message_remaining_.begin(),
                                    this->tx_message_remaining_.begin() + chunk_len);

  if (more_after_this) {
    this->parent_->run_later([this]() { this->send_next_fragment_(); });
  } else {
    SET_STATE(FsmState::WAITING_NOTIFICATION);
  }

  return true;
}

void EnergomeraBleComponent::begin_response_reads_(uint8_t slot_count) {
  uint8_t slots = slot_count + 1;
  if (slots == 0)
    slots = 1;

  if (slots > RX_HANDLES_NUM)
    slots = RX_HANDLES_NUM;

  this->expected_response_slots_ = slots;
  this->current_response_slot_ = 0;
  this->response_buffer_.clear();

  SET_STATE(FsmState::READING_RESPONSE);
  this->issue_next_response_read_();
}

void EnergomeraBleComponent::issue_next_response_read_() {
  if (this->current_response_slot_ >= this->expected_response_slots_) {
    this->finalize_command_response_();
    return;
  }

  uint16_t handle = this->ch_handles_rx_[this->current_response_slot_];
  if (handle == 0) {
    ESP_LOGW(TAG, "Response characteristic index %u not resolved", this->current_response_slot_);
    this->current_response_slot_++;
    this->issue_next_response_read_();
    return;
  }

  esp_err_t status = esp_ble_gattc_read_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), handle,
                                             ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_OK) {
    ESP_LOGW(TAG, "Failed to request response read (handle 0x%04X): %d", handle, status);
    SET_STATE(FsmState::ERROR);
  }
}

void EnergomeraBleComponent::handle_command_read_(const esp_ble_gattc_cb_param_t::gattc_read_char_evt_param &param) {
  if (param.status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "Response read failed (handle 0x%04X): %d", param.handle, param.status);
    SET_STATE(FsmState::ERROR);
    return;
  }

  for (uint16_t i = 0; i < param.value_len; i++) {
    this->response_buffer_.push_back(param.value[i] & 0x7F);
  }

  this->current_response_slot_++;
  this->issue_next_response_read_();
}

void EnergomeraBleComponent::finalize_command_response_() {
  SET_STATE(FsmState::GOT_RESPONSE);

  if (this->response_buffer_.empty()) {
    ESP_LOGW(TAG, "No response payload received");
    return;
  }

  ESP_LOGV(TAG, "RX: %s", format_frame_pretty(this->response_buffer_.data(), this->response_buffer_.size()).c_str());
  ESP_LOGVV(TAG, "RX: %s", format_hex_pretty(this->response_buffer_.data(), this->response_buffer_.size()).c_str());
}

void EnergomeraBleComponent::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                                 esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      if (!this->parent_->check_addr(param->connect.remote_bda))
        break;

      this->ch_handle_tx_ = 0;
      this->ch_handle_cccd_ = 0;
      memset(this->ch_handles_rx_, 0, sizeof(this->ch_handles_rx_));
      this->tx_fragment_started_ = false;
      this->tx_sequence_counter_ = 0;
      this->tx_message_remaining_.clear();
      this->response_buffer_.clear();
      this->expected_response_slots_ = 0;
      this->current_response_slot_ = 0;

      break;
    }

    case ESP_GATTC_OPEN_EVT: {
      if (!this->parent_->check_addr(param->open.remote_bda))
        break;
      if (param->open.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Failed to open GATT connection: status=%d", param->open.status);
        SET_STATE(FsmState::ERROR);
      } else {
        esp_ble_gattc_send_mtu_req(this->parent_->get_gattc_if(), this->parent_->get_conn_id());
      }
      break;
    }

    case ESP_GATTC_CFG_MTU_EVT: {
      if (param->cfg_mtu.status == ESP_GATT_OK) {
        this->mtu_ = param->cfg_mtu.mtu;
        ESP_LOGVV(TAG, "MTU set to %d", param->cfg_mtu.mtu);
      }
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      if (param->search_cmpl.conn_id != this->parent_->get_conn_id())
        break;
      ESP_LOGVV(TAG, "ESP_GATTC_SEARCH_CMPL_EVT: connected=%s, paired=%s", this->parent_->connected() ? "YES" : "NO",
                this->parent_->is_paired() ? "YES" : "NO");

      if (!this->discover_characteristics_()) {
        SET_STATE(FsmState::ERROR);
        this->parent_->disconnect();
        break;
      }

      // TODO: check if characteristics not found - we need to disconnect, remove bond, retry connection it will re-pair

      esp_err_t status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(),
                                                           this->parent_->get_remote_bda(), this->ch_handle_tx_);
      if (status != ESP_OK) {
        ESP_LOGW(TAG, "esp_ble_gattc_register_for_notify failed: %d (continuing)", status);
      }

      uint16_t notify_en = 0x0001;
      auto err = esp_ble_gattc_write_char_descr(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                                this->ch_handle_cccd_, sizeof(notify_en), (uint8_t *) &notify_en,
                                                ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);

      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enable notifications on handle 0x%04X: %d", this->ch_handle_cccd_, err);
        SET_STATE(FsmState::ERROR);
        return;
      }
      SET_STATE(FsmState::WAITING_NOTIFICATION_ENABLE);

    } break;

    case ESP_GATTC_READ_CHAR_EVT: {
      if (param->read.conn_id != this->parent_->get_conn_id())
        break;

      if (this->state_ == FsmState::READING_RESPONSE)
        this->handle_command_read_(param->read);
      break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT: {
      if (param->write.conn_id != this->parent_->get_conn_id())
        break;
      ESP_LOGVV(TAG, "ESP_GATTC_WRITE_DESCR_EVT received (handle = 0x%04X, status=%d)", param->write.handle,
                param->write.status);

      if (this->state_ == FsmState::WAITING_NOTIFICATION_ENABLE) {
        if (param->write.status == ESP_GATT_OK) {
          ESP_LOGI(TAG, "Notifications enabled");
          SET_STATE(FsmState::PREPARING_COMMAND);
        } else {
          ESP_LOGW(TAG, "Failed to enable notifications: %d", param->write.status);
          SET_STATE(FsmState::ERROR);
        }
      }
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent_->get_conn_id())
        break;

      ESP_LOGV(TAG, "Notification received (handle = 0x%04X, )", param->notify.handle, param->notify.value_len);
      this->expected_response_slots_ = param->notify.value[0];

      if (!param->notify.is_notify)
        break;

      if (this->state_ != FsmState::WAITING_NOTIFICATION)
        break;

      if (!param->notify.value || param->notify.value_len == 0) {
        ESP_LOGW(TAG, "Notification with empty payload received");
        break;
      }

      uint8_t slot_count = param->notify.value[0];
      ESP_LOGV(TAG, "Notification received (%u chunks to read)", slot_count);
      this->begin_response_reads_(slot_count);
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      if (!this->parent_->check_addr(param->disconnect.remote_bda))
        break;
      ESP_LOGVV(TAG, "GATT client disconnected: reason=0x%02X", param->disconnect.reason);

      this->ch_handle_tx_ = 0;
      this->ch_handle_cccd_ = 0;
      memset(this->ch_handles_rx_, 0, sizeof(this->ch_handles_rx_));

      this->tx_fragment_started_ = false;
      this->tx_sequence_counter_ = 0;
      this->mtu_ = 23;
      this->tx_message_remaining_.clear();
      this->response_buffer_.clear();
      this->expected_response_slots_ = 0;
      this->current_response_slot_ = 0;

      if (this->state_ != FsmState::PUBLISH) {
        SET_STATE(FsmState::IDLE);
      }
      break;
    }
    default:
      break;
  }
}

void EnergomeraBleComponent::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr)) {
        break;
      }
      this->pin_code_was_requested_ = true;
      ESP_LOGE(TAG, "*** Passkey request - supplying PIN %06u ***", this->passkey_);
      esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, this->passkey_);
      break;

    case ESP_GAP_BLE_SEC_REQ_EVT: {
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr)) {
        break;
      }
      auto auth_cmpl = param->ble_security.auth_cmpl;
      ESP_LOGI(TAG, "ESP_GAP_BLE_SEC_REQ_EVT success: %d, fail reason: %d, auth mode: %d", auth_cmpl.success,
               auth_cmpl.fail_reason, auth_cmpl.auth_mode);
      ESP_LOGW(TAG, "*** Security request received, responding... ***");
      esp_err_t sec_rsp = esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      ESP_LOGI(TAG, "Security response result: %d", sec_rsp);
      break;
    }

    case ESP_GAP_BLE_NC_REQ_EVT:
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr)) {
        break;
      }
      ESP_LOGE(TAG, "*** Numeric comparison request: %06u ***", param->ble_security.key_notif.passkey);
      esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
      break;

    case ESP_GAP_BLE_OOB_REQ_EVT: {
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr)) {
        break;
      }
      ESP_LOGE(TAG, "*** OOB data request - rejecting ***");
      esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, nullptr, 16);
      break;
    }

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
      if (!this->parent_->check_addr(param->ble_security.auth_cmpl.bd_addr)) {
        break;
      }

      if (param->ble_security.auth_cmpl.success) {
        ESP_LOGVV(TAG, "Pairing completed successfully. Did we tell PIN to device ? %s ***",
                  this->pin_code_was_requested_ ? "YES" : "NO");

      } else {
        ESP_LOGE(TAG, "*** Pairing FAILED, reason=%d ***", param->ble_security.auth_cmpl.fail_reason);
      }
      break;
    default:
      break;
  }
}

const char *EnergomeraBleComponent::state_to_string_(FsmState state) const {
  switch (state) {
    case FsmState::NOT_INITIALIZED:
      return "NOT_INITIALIZED";
    case FsmState::IDLE:
      return "IDLE";
    case FsmState::START:
      return "START";
    case FsmState::RESOLVING:
      return "RESOLVING";
    case FsmState::REQUESTING_FIRMWARE:
      return "REQUESTING_FIRMWARE";
    case FsmState::WAITING_FIRMWARE:
      return "WAITING_FIRMWARE";
    case FsmState::ENABLING_NOTIFICATION:
      return "ENABLING_NOTIFICATION";
    case FsmState::WAITING_NOTIFICATION_ENABLE:
      return "WAITING_NOTIFICATION_ENABLE";
    case FsmState::PREPARING_COMMAND:
      return "PREPARING_COMMAND";
    case FsmState::SENDING_COMMAND:
      return "SENDING_COMMAND";
    case FsmState::WAITING_NOTIFICATION:
      return "WAITING_NOTIFICATION";
    case FsmState::READING_RESPONSE:
      return "READING_RESPONSE";
    case FsmState::GOT_RESPONSE:
      return "GOT_RESPONSE";
    case FsmState::PUBLISH:
      return "PUBLISH";
    case FsmState::ERROR:
      return "ERROR";
    case FsmState::DISCONNECTED:
      return "DISCONNECTED";
    default:
      return "UNKNOWN";
  }
}

void EnergomeraBleComponent::set_state_(FsmState state) {
  ESP_LOGV(TAG, "State change: %s -> %s", state_to_string_(this->state_), state_to_string_(state));
  this->state_ = state;
}

}  // namespace energomera_ble
}  // namespace esphome
