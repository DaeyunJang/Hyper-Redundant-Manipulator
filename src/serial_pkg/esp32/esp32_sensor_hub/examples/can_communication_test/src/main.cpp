#include <Arduino.h>
#include <driver/twai.h>

// ESP32-S3 <-> SN65HVD230: GPIO1 -> CTX, GPIO2 <- CRX.
constexpr gpio_num_t kCanTxPin = GPIO_NUM_1;
constexpr gpio_num_t kCanRxPin = GPIO_NUM_2;

// Same F/T sensor CAN configuration as the working Arduino/MCP2515 code.
constexpr uint16_t kForceId = 0x2A;
constexpr uint16_t kTorqueId = 0x2B;
constexpr uint32_t kSerialBaudrate = 115200;

struct ForceTorqueFrame {
  int32_t fx = 0;
  int32_t fy = 0;
  int32_t fz = 0;
  int32_t tx = 0;
  int32_t ty = 0;
  int32_t tz = 0;
};

ForceTorqueFrame gFt;

void PrintCanStatus() {
  static uint32_t previousMillis = 0;
  const uint32_t now = millis();
  if (now - previousMillis < 1000) {
    return;
  }

  previousMillis = now;
  twai_status_info_t status = {};
  if (twai_get_status_info(&status) == ESP_OK) {
    Serial.printf("[CAN] state=%d rx=%lu rx_err=%lu tx_err=%lu bus_err=%lu\n",
                  static_cast<int>(status.state),
                  static_cast<unsigned long>(status.msgs_to_rx),
                  static_cast<unsigned long>(status.rx_error_counter),
                  static_cast<unsigned long>(status.tx_error_counter),
                  static_cast<unsigned long>(status.bus_error_count));
  }
}

int32_t DecodeAxis(const uint8_t* data) {
  const uint16_t raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  return static_cast<int32_t>(raw) - 30000;
}

void SendSerial() {
  // Legacy protocol: /Fx,Fy,Fz,Tx,Ty,Tz;
  Serial.write('/');
  Serial.printf("%ld,%ld,%ld,%ld,%ld,%ld", static_cast<long>(gFt.fx),
                static_cast<long>(gFt.fy), static_cast<long>(gFt.fz),
                static_cast<long>(gFt.tx), static_cast<long>(gFt.ty),
                static_cast<long>(gFt.tz));
  Serial.println(';');
}

void ParseFts(const twai_message_t& message) {
  // The Arduino code reads three 16-bit big-endian axes from bytes 0..5.
  if (message.extd || message.rtr || message.data_length_code < 6) {
    return;
  }

  if (message.identifier == kForceId) {
    gFt.fx = DecodeAxis(&message.data[0]);
    gFt.fy = DecodeAxis(&message.data[2]);
    gFt.fz = DecodeAxis(&message.data[4]);
    SendSerial();
  } else if (message.identifier == kTorqueId) {
    gFt.tx = DecodeAxis(&message.data[0]);
    gFt.ty = DecodeAxis(&message.data[2]);
    gFt.tz = DecodeAxis(&message.data[4]);
    SendSerial();
  }
}

bool InitCan() {
  twai_general_config_t general =
      TWAI_GENERAL_CONFIG_DEFAULT(kCanTxPin, kCanRxPin, TWAI_MODE_NORMAL);
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t result = twai_driver_install(&general, &timing, &filter);
  if (result != ESP_OK) {
    Serial.printf("TWAI install failed: %s\n", esp_err_to_name(result));
    return false;
  }

  result = twai_start();
  if (result != ESP_OK) {
    Serial.printf("TWAI start failed: %s\n", esp_err_to_name(result));
    twai_driver_uninstall();
    return false;
  }

  return true;
}

void setup() {
  Serial.begin(kSerialBaudrate);
  delay(1000);

  if (!InitCan()) {
    while (true) {
      delay(1000);
    }
  }

  Serial.println("start CAN communication: 1 Mbps, IDs 0x2A / 0x2B");
}

void loop() {
  twai_message_t message = {};
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    Serial.printf("[RAW] id=0x%03lX dlc=%u\n",
                  static_cast<unsigned long>(message.identifier),
                  message.data_length_code);
    ParseFts(message);
  }

  PrintCanStatus();
}
