#include <Arduino.h>  // HX711 sampling implementation.
#include <HX711_ADC.h>

extern portMUX_TYPE gDataMutex;
extern float gHx711Data[4];

namespace
{
constexpr uint8_t HX711_COUNT = 4;
constexpr int DOUT_PINS[HX711_COUNT] = {4, 6, 15, 17};
constexpr int SCK_PINS[HX711_COUNT] = {5, 7, 16, 18};

// Values for channels 0 and 1 are from the previous Arduino firmware.
constexpr float CAL_FACTORS[HX711_COUNT] = {275.684936f, 255.5f, 1.0f, 1.0f};
constexpr float OUTPUT_OFFSETS[HX711_COUNT] = {30670.0f, 32770.0f, 0.0f, 0.0f};

// Set USE_MOVING_AVERAGE to false for fastest response, or lower the sample
// count (1, 2, 4, 8, or 16) to reduce smoothing.
constexpr bool USE_MOVING_AVERAGE = true;
constexpr int MOVING_AVERAGE_SAMPLES = 16;

HX711_ADC scales[HX711_COUNT] = {
    HX711_ADC(DOUT_PINS[0], SCK_PINS[0]),
    HX711_ADC(DOUT_PINS[1], SCK_PINS[1]),
    HX711_ADC(DOUT_PINS[2], SCK_PINS[2]),
    HX711_ADC(DOUT_PINS[3], SCK_PINS[3]),
};
}  // namespace

void hx711Begin()
{
    for (uint8_t channel = 0; channel < HX711_COUNT; ++channel)
    {
        scales[channel].begin();
        scales[channel].start(2000, false);  // Stabilize; do not tare automatically.
        scales[channel].setCalFactor(CAL_FACTORS[channel]);
        // HX711_ADC uses a moving-average dataset. With filtering disabled,
        // one sample is used (no averaging; only the library's peak rejection remains).
        const int samples = USE_MOVING_AVERAGE ? MOVING_AVERAGE_SAMPLES : 1;
        scales[channel].setSamplesInUse(samples);
    }
}

void hx711Poll()
{
    for (uint8_t channel = 0; channel < HX711_COUNT; ++channel)
    {
        if (!scales[channel].update())
        {
            continue;
        }

        const float value = scales[channel].getData() - OUTPUT_OFFSETS[channel];
        portENTER_CRITICAL(&gDataMutex);
        gHx711Data[channel] = value;
        portEXIT_CRITICAL(&gDataMutex);
    }
}
