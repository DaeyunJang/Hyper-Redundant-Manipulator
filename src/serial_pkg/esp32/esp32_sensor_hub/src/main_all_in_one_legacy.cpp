#include <Arduino.h>
#include "HX711.h"

// Snapshot of the pre-module single-file implementation. Excluded from builds.
constexpr uint32_t SERIAL_BAUD = 921600;
constexpr uint32_t HX711_PERIOD_US = 12500;
constexpr uint8_t SAMPLE_QUEUE_LENGTH = 64;
constexpr uint8_t HX711_COUNT = 4;
const int HX711_DOUT_PINS[HX711_COUNT] = {4, 6, 15, 17};
const int HX711_SCK_PINS[HX711_COUNT] = {5, 7, 16, 18};

struct SensorSample
{
    uint8_t sensorId;
    int32_t raw;
    uint32_t timestampUs;
    uint32_t dtUs;
};

HX711 gScales[HX711_COUNT];
bool gScaleReady[HX711_COUNT] = {};
uint32_t gNextDueUs[HX711_COUNT] = {};
uint32_t gLastSampleUs[HX711_COUNT] = {};
QueueHandle_t gSampleQueue = nullptr;

void publishSample(const SensorSample &sample)
{
    const float hzInst = sample.dtUs ? 1000000.0f / sample.dtUs : 0.0f;
    Serial.printf("S%u raw=%ld dt_us=%lu hz_inst=%.2f\n", sample.sensorId,
                  static_cast<long>(sample.raw), static_cast<unsigned long>(sample.dtUs), hzInst);
}

void loopHx711()
{
    const uint32_t nowUs = micros();
    for (uint8_t i = 0; i < HX711_COUNT; ++i)
    {
        if (!gScaleReady[i] || static_cast<int32_t>(nowUs - gNextDueUs[i]) < 0)
        {
            continue;
        }
        gNextDueUs[i] += HX711_PERIOD_US;
        if (!gScales[i].is_ready())
        {
            continue;
        }
        SensorSample sample = {i, static_cast<int32_t>(gScales[i].read()), micros(), 0};
        sample.dtUs = gLastSampleUs[i] ? sample.timestampUs - gLastSampleUs[i] : 0;
        gLastSampleUs[i] = sample.timestampUs;
        xQueueSend(gSampleQueue, &sample, 0);
    }
}

void loopCan() {}

void loopPcTx()
{
    SensorSample sample = {};
    if (xQueueReceive(gSampleQueue, &sample, 0) == pdPASS)
    {
        publishSample(sample);
    }
}

void hx711Task(void *)
{
    for (;;)
    {
        loopHx711();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void canTask(void *)
{
    for (;;)
    {
        loopCan();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void pcTxTask(void *)
{
    for (;;)
    {
        loopPcTx();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    gSampleQueue = xQueueCreate(SAMPLE_QUEUE_LENGTH, sizeof(SensorSample));
    for (uint8_t i = 0; i < HX711_COUNT; ++i)
    {
        gScales[i].begin(HX711_DOUT_PINS[i], HX711_SCK_PINS[i]);
        gScaleReady[i] = gScales[i].wait_ready_timeout(500);
        gNextDueUs[i] = micros();
    }
    xTaskCreate(hx711Task, "Hx711Task", 4096, nullptr, 3, nullptr);
    xTaskCreate(canTask, "CanTask", 2048, nullptr, 2, nullptr);
    xTaskCreate(pcTxTask, "PcTxTask", 2048, nullptr, 1, nullptr);
}

void loop()
{
    vTaskDelay(portMAX_DELAY);
}
