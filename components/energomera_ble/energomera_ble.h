#pragma once

#include "esphome/components/ble_client/ble_client.h"
#include "esphome/core/component.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

#include <esp_gattc_api.h>

#include <array>
#include <map>
#include <string>
#include <vector>

#include "energomera_ble_sensor.h"

namespace esphome {
namespace energomera_ble {

const uint8_t VAL_NUM = 12;
using ValueRefsArray = std::array<char *, VAL_NUM>;
using SensorMap = std::multimap<std::string, EnergomeraBleSensorBase *>;

using ble_defer_fn_t = std::function<void()>;  // NOLINT

static constexpr size_t RX_HANDLES_NUM = 15;
static constexpr size_t TX_BUFFER_SIZE = 64;
static constexpr size_t RX_BUFFER_SIZE = 256;

static constexpr size_t DESIRED_MTU = 64;

class EnergomeraBleComponent : public PollingComponent, public ble_client::BLEClientNode {
 public:
  EnergomeraBleComponent(){};

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_BLUETOOTH; };

  void set_passkey(uint32_t passkey) { this->passkey_ = passkey % 1000000U; };

  void set_meter_address(const std::string &address) { (void) address; }
  void set_receive_timeout_ms(uint32_t timeout) { (void) timeout; }
  void set_delay_between_requests_ms(uint32_t delay) { (void) delay; }
  void set_reboot_after_failure(uint16_t value) { (void) value; }
  void set_signal_strength(sensor::Sensor *signal_strength) { signal_strength_ = signal_strength; }

  void register_sensor(EnergomeraBleSensorBase *sensor);

  void try_connect();
  void remove_bonding();

#ifdef USE_BINARY_SENSOR
  void set_indicator(binary_sensor::BinarySensor *indicator) { (void) indicator; }
#endif
#ifdef USE_TIME
  void set_time_source(time::RealTimeClock *time) { (void) time; }
#endif

 protected:
  bool set_sensor_value_(EnergomeraBleSensorBase *sensor, ValueRefsArray &vals);

  bool ble_discover_characteristics_();


  enum class FsmState : uint8_t {
    NOT_INITIALIZED = 0,
    IDLE,
    STARTING,
    PREPARING_COMMAND,
    SENDING_COMMAND,
    READING_RESPONSE,
    GOT_RESPONSE,
    PUBLISH,
    ERROR,
    DISCONNECTED
  };
  FsmState state_{FsmState::NOT_INITIALIZED};

  const char *state_to_string_(FsmState state) const;
  void set_state_(FsmState state);

  void prepare_request_frame_(const std::string &request);
  void send_next_fragment_();

  // BLE send/receive
  uint16_t get_max_payload_() const;
  void run_in_ble_thread_(const ble_defer_fn_t &fn);
  ble_defer_fn_t ble_defer_fn_{nullptr};

  void ble_set_error_();

  // BLE Thread functions
  bool ble_send_next_fragment_();
  void ble_initiate_fragment_reads_(uint8_t slots);
  void ble_request_next_fragment_();
  void ble_read_fragment_(const esp_ble_gattc_cb_param_t::gattc_read_char_evt_param &param);

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

  uint16_t mtu_{23};

  union {
    uint8_t raw;
    struct {
      bool notifications_enabled : 1;
      bool pin_code_was_requested : 1;
      bool tx_error : 1;
      bool rx_reply : 1;
    };
  } flags_{0};

  // Handles
  uint16_t ch_version_{0};
  uint16_t ch_handle_tx_{0};
  uint16_t ch_handle_cccd_{0};
  uint16_t ch_handles_rx_[RX_HANDLES_NUM]{0};

  uint8_t tx_buffer_[TX_BUFFER_SIZE];
  uint8_t *tx_ptr_{nullptr};
  uint8_t tx_data_remaining_{0};
  uint8_t tx_packet_size_{19};

  uint8_t tx_sequence_counter_{0};
  bool tx_fragment_started_{false};

  uint8_t rx_buffer_[RX_BUFFER_SIZE];
  size_t rx_len_{0};

  int8_t rssi_{0};

 
  uint8_t rx_fragments_expected_{0};
  uint8_t rx_current_fragment_{0};

  uint32_t passkey_{0};

  SensorMap sensors_{};
  SensorMap::iterator request_iter{nullptr};
  SensorMap::iterator sensor_iter{nullptr};
  sensor::Sensor *signal_strength_{nullptr};

  void internal_safeguard_();

  char *get_nth_value_from_csv_(char *line, uint8_t idx);
  uint8_t get_values_from_brackets_(char *line, ValueRefsArray &vals);
};

}  // namespace energomera_ble
}  // namespace esphome
