#include <Arduino.h>
#include "driver/twai.h"

// ============================================================
// CAN configuration
// ============================================================

// ESP32-S3 -> SN65HVD230
#define CAN_TX_PIN GPIO_NUM_1     // GPIO1 -> CTX
#define CAN_RX_PIN GPIO_NUM_2     // GPIO2 <- CRX

// 기존 F/T Sensor CAN ID
#define FORCE_CAN_ID   0x2A
#define TORQUE_CAN_ID  0x2B

// Serial
#define SERIAL_BAUDRATE 921600

// Debug options
#define PRINT_RAW_CAN       1
#define PRINT_DECODED_FT    1


// ============================================================
// F/T Sensor Variables
// ============================================================

// Force [mN]
volatile int32_t fx = 0;
volatile int32_t fy = 0;
volatile int32_t fz = 0;

// Torque [mNm]
volatile int32_t tx = 0;
volatile int32_t ty = 0;
volatile int32_t tz = 0;


// ============================================================
// CAN initialization
// ============================================================

bool initCAN()
{
    Serial.println();
    Serial.println("======================================");
    Serial.println("Initializing ESP32-S3 TWAI");
    Serial.println("CAN bitrate : 1 Mbps");
    Serial.println("TX GPIO     : GPIO1");
    Serial.println("RX GPIO     : GPIO2");
    Serial.println("Mode        : NORMAL");
    Serial.println("Filter      : ACCEPT ALL");
    Serial.println("======================================");

    // --------------------------------------------------------
    // General configuration
    //
    // NORMAL:
    // F/T sensor가 전송한 frame에 ESP32가 ACK 응답함.
    // 센서와 ESP32만 있는 CAN bus에서는 NORMAL을 권장.
    // --------------------------------------------------------

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(
            CAN_TX_PIN,
            CAN_RX_PIN,
            TWAI_MODE_NORMAL
        );

    // CAN frame이 빠르게 들어오는 경우를 대비
    g_config.rx_queue_len = 64;

    // --------------------------------------------------------
    // Bitrate = 1 Mbps
    // --------------------------------------------------------

    twai_timing_config_t t_config =
        TWAI_TIMING_CONFIG_1MBITS();

    // --------------------------------------------------------
    // 모든 Standard/Extended CAN frame 수신
    // --------------------------------------------------------

    twai_filter_config_t f_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();


    // --------------------------------------------------------
    // Driver install
    // --------------------------------------------------------

    esp_err_t result =
        twai_driver_install(
            &g_config,
            &t_config,
            &f_config
        );

    if (result != ESP_OK)
    {
        Serial.printf(
            "[ERROR] TWAI driver install failed: %s\n",
            esp_err_to_name(result)
        );

        return false;
    }

    Serial.println("[OK] TWAI driver installed");


    // --------------------------------------------------------
    // Enable useful alerts
    // --------------------------------------------------------

    uint32_t alerts =
        TWAI_ALERT_RX_QUEUE_FULL |
        TWAI_ALERT_BUS_ERROR |
        TWAI_ALERT_ERR_PASS |
        TWAI_ALERT_BUS_OFF;

    twai_reconfigure_alerts(alerts, nullptr);


    // --------------------------------------------------------
    // Start TWAI
    // --------------------------------------------------------

    result = twai_start();

    if (result != ESP_OK)
    {
        Serial.printf(
            "[ERROR] TWAI start failed: %s\n",
            esp_err_to_name(result)
        );

        twai_driver_uninstall();

        return false;
    }

    Serial.println("[OK] TWAI started");
    Serial.println();

    return true;
}


// ============================================================
// Print raw CAN message
// ============================================================

void printRawCAN(const twai_message_t &msg)
{
#if PRINT_RAW_CAN

    Serial.printf(
        "[%10lu us] ",
        (unsigned long)micros()
    );


    // Standard / Extended
    if (msg.extd)
    {
        Serial.printf(
            "EXT ID=0x%08lX ",
            (unsigned long)msg.identifier
        );
    }
    else
    {
        Serial.printf(
            "STD ID=0x%03lX ",
            (unsigned long)msg.identifier
        );
    }


    // RTR
    if (msg.rtr)
    {
        Serial.print("RTR ");
    }
    else
    {
        Serial.print("DATA ");
    }


    Serial.printf(
        "DLC=%d ",
        msg.data_length_code
    );


    // Data bytes
    Serial.print("[");

    if (!msg.rtr)
    {
        for (uint8_t i = 0;
             i < msg.data_length_code;
             i++)
        {
            Serial.printf("%02X", msg.data[i]);

            if (i < msg.data_length_code - 1)
            {
                Serial.print(" ");
            }
        }
    }

    Serial.println("]");

#endif
}


// ============================================================
// F/T Sensor decoding
// ============================================================

void decodeFTSensor(const twai_message_t &msg)
{
    // F/T frame은 최소 6 byte 필요
    if (msg.data_length_code < 6)
    {
        return;
    }


    // --------------------------------------------------------
    // FORCE
    // CAN ID = 0x2A
    //
    // 기존 Arduino 코드와 동일:
    //
    // Fx = data[0:1] - 30000
    // Fy = data[2:3] - 30000
    // Fz = data[4:5] - 30000
    //
    // Unit = mN
    // --------------------------------------------------------

    if (msg.identifier == FORCE_CAN_ID)
    {
        fx =
            ((uint16_t)msg.data[0] << 8 |
             (uint16_t)msg.data[1])
            - 30000;

        fy =
            ((uint16_t)msg.data[2] << 8 |
             (uint16_t)msg.data[3])
            - 30000;

        fz =
            ((uint16_t)msg.data[4] << 8 |
             (uint16_t)msg.data[5])
            - 30000;


#if PRINT_DECODED_FT

        Serial.printf(
            "    FORCE  "
            "Fx=%ld mN, "
            "Fy=%ld mN, "
            "Fz=%ld mN\n",
            (long)fx,
            (long)fy,
            (long)fz
        );

#endif
    }


    // --------------------------------------------------------
    // TORQUE
    // CAN ID = 0x2B
    //
    // Tx = data[0:1] - 30000
    // Ty = data[2:3] - 30000
    // Tz = data[4:5] - 30000
    //
    // Unit = mNm
    // --------------------------------------------------------

    else if (msg.identifier == TORQUE_CAN_ID)
    {
        tx =
            ((uint16_t)msg.data[0] << 8 |
             (uint16_t)msg.data[1])
            - 30000;

        ty =
            ((uint16_t)msg.data[2] << 8 |
             (uint16_t)msg.data[3])
            - 30000;

        tz =
            ((uint16_t)msg.data[4] << 8 |
             (uint16_t)msg.data[5])
            - 30000;


#if PRINT_DECODED_FT

        Serial.printf(
            "    TORQUE "
            "Tx=%ld mNm, "
            "Ty=%ld mNm, "
            "Tz=%ld mNm\n",
            (long)tx,
            (long)ty,
            (long)tz
        );

#endif
    }
}


// ============================================================
// CAN status / error check
// ============================================================

void checkCANStatus()
{
    uint32_t alerts = 0;

    // Non-blocking
    if (twai_read_alerts(
            &alerts,
            0) != ESP_OK)
    {
        return;
    }


    if (alerts == 0)
    {
        return;
    }


    twai_status_info_t status;

    twai_get_status_info(&status);


    if (alerts & TWAI_ALERT_RX_QUEUE_FULL)
    {
        Serial.println();
        Serial.println(
            "[WARNING] CAN RX queue FULL"
        );

        Serial.printf(
            "RX queued  : %lu\n",
            (unsigned long)status.msgs_to_rx
        );

        Serial.printf(
            "RX missed  : %lu\n",
            (unsigned long)status.rx_missed_count
        );

        Serial.printf(
            "RX overrun : %lu\n",
            (unsigned long)status.rx_overrun_count
        );
    }


    if (alerts & TWAI_ALERT_BUS_ERROR)
    {
        Serial.println();
        Serial.println(
            "[WARNING] CAN BUS ERROR"
        );

        Serial.printf(
            "Bus error count : %lu\n",
            (unsigned long)status.bus_error_count
        );

        Serial.printf(
            "RX error count  : %lu\n",
            (unsigned long)status.rx_error_counter
        );

        Serial.printf(
            "TX error count  : %lu\n",
            (unsigned long)status.tx_error_counter
        );
    }


    if (alerts & TWAI_ALERT_ERR_PASS)
    {
        Serial.println();
        Serial.println(
            "[WARNING] TWAI entered ERROR PASSIVE"
        );
    }


    if (alerts & TWAI_ALERT_BUS_OFF)
    {
        Serial.println();
        Serial.println(
            "[ERROR] TWAI BUS OFF"
        );
    }
}


// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);

    delay(1500);

    Serial.println();
    Serial.println();
    Serial.println("======================================");
    Serial.println("ESP32-S3 F/T CAN Sensor Test");
    Serial.println("======================================");


    if (!initCAN())
    {
        Serial.println();
        Serial.println(
            "CAN initialization FAILED."
        );

        while (true)
        {
            delay(1000);
        }
    }


    Serial.println();
    Serial.println("Waiting for CAN frames...");
    Serial.println();
}


// ============================================================
// Main loop
// ============================================================

void loop()
{
    twai_message_t msg;


    // --------------------------------------------------------
    // Receive CAN message
    //
    // 최대 10 ms 기다림
    // --------------------------------------------------------

    esp_err_t result =
        twai_receive(
            &msg,
            pdMS_TO_TICKS(10)
        );


    if (result == ESP_OK)
    {
        // 모든 raw CAN frame 표시
        printRawCAN(msg);

        // F/T Sensor frame이면 값 변환
        decodeFTSensor(msg);
    }


    // CAN error 확인
    checkCANStatus();
}