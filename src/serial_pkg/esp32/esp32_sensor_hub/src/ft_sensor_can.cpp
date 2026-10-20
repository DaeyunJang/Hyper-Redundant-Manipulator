#include <Arduino.h>
#include <driver/twai.h>

// ============================================================
// Shared F/T sensor data
//
// Force  : mN
// Torque : mNm
// ============================================================

extern portMUX_TYPE gDataMutex;

extern float gFx;
extern float gFy;
extern float gFz;

extern float gTx;
extern float gTy;
extern float gTz;


namespace
{

constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_1;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_2;

constexpr uint16_t FORCE_CAN_ID  = 0x1A;
constexpr uint16_t TORQUE_CAN_ID = 0x1B;

bool canReady = false;


// ============================================================
// Raw CAN data decoding
//
// AFT20 sends each axis as unsigned 16-bit big-endian data.
//
//   data[0] : MSB
//   data[1] : LSB
//
// ============================================================

uint16_t decodeRaw(const uint8_t *data)
{
    return
        (static_cast<uint16_t>(data[0]) << 8) |
         static_cast<uint16_t>(data[1]);
}


// ============================================================
// Force decoding
//
// AFT20 specification:
//
//   Force [N] = raw / 1000 - 30
//
// Convert to mN:
//
//   Force [mN]
//     = (raw / 1000 - 30) * 1000
//     = raw - 30000
//
// Therefore:
//
//   1 count = 1 mN
//
// ============================================================

float decodeForce_mN(const uint8_t *data)
{
    const uint16_t raw = decodeRaw(data);

    return static_cast<float>(raw) - 30000.0f;
}


// ============================================================
// Torque decoding
//
// AFT20 specification:
//
//   Torque [Nm] = raw / 100000 - 0.3
//
// Convert to mNm:
//
//   Torque [mNm]
//     = (raw / 100000 - 0.3) * 1000
//     = (raw - 30000) / 100
//
// Therefore:
//
//   1 count = 0.01 mNm
//
// ============================================================

float decodeTorque_mNm(const uint8_t *data)
{
    const uint16_t raw = decodeRaw(data);

    return
        (static_cast<float>(raw) - 30000.0f) / 100.0f;
}

} // namespace


// ============================================================
// Initialize CAN interface
// ============================================================

void ftCanBegin()
{
    twai_general_config_t general =
        TWAI_GENERAL_CONFIG_DEFAULT(
            CAN_TX_PIN,
            CAN_RX_PIN,
            TWAI_MODE_NORMAL
        );

    twai_timing_config_t timing =
        TWAI_TIMING_CONFIG_1MBITS();

    twai_filter_config_t filter =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();


    const esp_err_t installResult =
        twai_driver_install(
            &general,
            &timing,
            &filter
        );

    if (installResult != ESP_OK)
    {
        canReady = false;
        return;
    }


    const esp_err_t startResult =
        twai_start();

    if (startResult != ESP_OK)
    {
        twai_driver_uninstall();
        canReady = false;
        return;
    }


    canReady = true;
}


// ============================================================
// Poll CAN messages
//
// FORCE_CAN_ID  (0x2A)
//   Fx, Fy, Fz
//   output unit = mN
//
// TORQUE_CAN_ID (0x2B)
//   Tx, Ty, Tz
//   output unit = mNm
//
// ============================================================

void ftCanPoll()
{
    if (!canReady)
    {
        return;
    }


    twai_message_t message = {};


    while (twai_receive(&message, 0) == ESP_OK)
    {
        // Ignore:
        //   - Extended CAN frames
        //   - RTR frames
        //   - Invalid/short packets
        if (message.extd ||
            message.rtr ||
            message.data_length_code < 6)
        {
            continue;
        }


        // ----------------------------------------------------
        // Force data
        // ----------------------------------------------------

        if (message.identifier == FORCE_CAN_ID)
        {
            const float fx =
                decodeForce_mN(&message.data[0]);

            const float fy =
                decodeForce_mN(&message.data[2]);

            const float fz =
                decodeForce_mN(&message.data[4]);


            portENTER_CRITICAL(&gDataMutex);

            gFx = fx;
            gFy = fy;
            gFz = fz;

            portEXIT_CRITICAL(&gDataMutex);
        }


        // ----------------------------------------------------
        // Torque data
        // ----------------------------------------------------

        else if (message.identifier == TORQUE_CAN_ID)
        {
            const float tx =
                decodeTorque_mNm(&message.data[0]);

            const float ty =
                decodeTorque_mNm(&message.data[2]);

            const float tz =
                decodeTorque_mNm(&message.data[4]);


            portENTER_CRITICAL(&gDataMutex);

            gTx = tx;
            gTy = ty;
            gTz = tz;

            portEXIT_CRITICAL(&gDataMutex);
        }
    }
}