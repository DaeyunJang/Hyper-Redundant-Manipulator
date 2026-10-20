# HX711 calibration

This is an independent PlatformIO project based on the `HX711_ADC` calibration example.

It calibrates one HX711 channel at a time. The default pins are DOUT `GPIO4` and SCK `GPIO5` (channel 0 of the main firmware). Change `HX711_DOUT` and `HX711_SCK` in `src/main.cpp` to calibrate another channel.

From this folder, upload and open the monitor:

```bash
pio run -t upload
pio device monitor -b 57600
```

Follow the prompts: send `t` to tare, place a known mass, then send its numeric weight. Copy the reported calibration factor into `CAL_FACTORS` in `../../src/hx711.cpp`.
