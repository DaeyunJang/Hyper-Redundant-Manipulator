#include <Arduino.h>

constexpr uint32_t SERIAL_BAUD = 921600;
constexpr uint32_t SERIAL_PERIOD_MS = 5;  // 200 Hz

// Latest sensor values. They are updated by the HX711 and CAN tasks and copied
// atomically by the serial task.
portMUX_TYPE gDataMutex = portMUX_INITIALIZER_UNLOCKED;
float gHx711Data[4] = {};
float gFx = 0.0f;
float gFy = 0.0f;
float gFz = 0.0f;
float gTx = 0.0f;
float gTy = 0.0f;
float gTz = 0.0f;

void hx711Begin();
void hx711Poll();
void ftCanBegin();
void ftCanPoll();

namespace
{
void hx711Task(void *)
{
    for (;;)
    {
        hx711Poll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void ftCanTask(void *)
{
    for (;;)
    {
        ftCanPoll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void serialTask(void *)
{
    for (;;)
    {
        float hx[4];
        float fx, fy, fz, tx, ty, tz;

        portENTER_CRITICAL(&gDataMutex);
        for (uint8_t i = 0; i < 4; ++i)
        {
            hx[i] = gHx711Data[i];
        }
        fx = gFx;
        fy = gFy;
        fz = gFz;
        tx = gTx;
        ty = gTy;
        tz = gTz;
        portEXIT_CRITICAL(&gDataMutex);

        // Legacy serial protocol, expanded from 8 to 10 fields.
        Serial.printf(
            "/%.0f,%.0f,%.0f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f;\n",
            fx, fy, fz,
            tx, ty, tz,
            hx[0], hx[1], hx[2], hx[3]
        );
        vTaskDelay(pdMS_TO_TICKS(SERIAL_PERIOD_MS));
    }
}
}  // namespace

void setup()
{
    Serial.begin(SERIAL_BAUD);
    hx711Begin();
    ftCanBegin();

    xTaskCreate(hx711Task, "Hx711", 4096, nullptr, 3, nullptr);
    xTaskCreate(ftCanTask, "FtCan", 3072, nullptr, 2, nullptr);
    xTaskCreate(serialTask, "Serial", 3072, nullptr, 1, nullptr);
}

void loop()
{
    vTaskDelay(portMAX_DELAY);
}
