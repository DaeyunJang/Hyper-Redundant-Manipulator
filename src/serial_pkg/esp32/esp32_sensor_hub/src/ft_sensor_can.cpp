#include <Arduino.h>  // F/T sensor CAN implementation.
#include <driver/twai.h>

extern portMUX_TYPE gDataMutex;
extern int32_t gFx;
extern int32_t gFy;
extern int32_t gFz;
extern int32_t gTx;
extern int32_t gTy;
extern int32_t gTz;

namespace
{
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_1;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_2;
constexpr uint16_t FORCE_CAN_ID = 0x2A;
constexpr uint16_t TORQUE_CAN_ID = 0x2B;
bool canReady = false;

int32_t decodeAxis(const uint8_t *data)
{
    return static_cast<int32_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]) - 30000;
}
}  // namespace

void ftCanBegin()
{
    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    canReady = (twai_driver_install(&general, &timing, &filter) == ESP_OK) && (twai_start() == ESP_OK);
}

void ftCanPoll()
{
    if (!canReady)
    {
        return;
    }

    twai_message_t message = {};
    while (twai_receive(&message, 0) == ESP_OK)
    {
        if (message.extd || message.rtr || message.data_length_code < 6)
        {
            continue;
        }

        portENTER_CRITICAL(&gDataMutex);
        if (message.identifier == FORCE_CAN_ID)
        {
            gFx = decodeAxis(&message.data[0]);
            gFy = decodeAxis(&message.data[2]);
            gFz = decodeAxis(&message.data[4]);
        }
        else if (message.identifier == TORQUE_CAN_ID)
        {
            gTx = decodeAxis(&message.data[0]);
            gTy = decodeAxis(&message.data[2]);
            gTz = decodeAxis(&message.data[4]);
        }
        portEXIT_CRITICAL(&gDataMutex);
    }
}
