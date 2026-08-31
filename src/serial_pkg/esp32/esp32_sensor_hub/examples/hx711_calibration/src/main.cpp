// HX711_ADC official Calibration example, configured for ESP32-S3 channel 0.
#include <HX711_ADC.h>

#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

constexpr int HX711_DOUT = 4;
constexpr int HX711_SCK = 5;
constexpr int CAL_FACTOR_EEPROM_ADDRESS = 0;

HX711_ADC loadCell(HX711_DOUT, HX711_SCK);
unsigned long lastPrintMs = 0;

void calibrate();
void changeSavedCalFactor();

void setup()
{
    Serial.begin(921600);
    delay(10);
    Serial.println();
    Serial.println("Starting HX711 calibration...");

    loadCell.begin();
    constexpr unsigned long STABILIZING_TIME_MS = 2000;
    constexpr bool TARE_ON_START = true;
    loadCell.start(STABILIZING_TIME_MS, TARE_ON_START);

    if (loadCell.getTareTimeoutFlag() || loadCell.getSignalTimeoutFlag())
    {
        Serial.println("Timeout: check MCU-to-HX711 wiring and pin assignments.");
        while (true)
        {
            delay(1000);
        }
    }

    loadCell.setCalFactor(1.0f);
    Serial.println("Startup is complete");
    while (!loadCell.update())
    {
    }
    calibrate();
}

void loop()
{
    static bool newDataReady = false;
    if (loadCell.update())
    {
        newDataReady = true;
    }

    if (newDataReady && millis() > lastPrintMs)
    {
        Serial.print("Load cell output: ");
        Serial.println(loadCell.getData());
        newDataReady = false;
        lastPrintMs = millis();
    }

    if (Serial.available() > 0)
    {
        const char command = static_cast<char>(Serial.read());
        if (command == 't')
        {
            loadCell.tareNoDelay();
        }
        else if (command == 'r')
        {
            calibrate();
        }
        else if (command == 'c')
        {
            changeSavedCalFactor();
        }
    }

    if (loadCell.getTareStatus())
    {
        Serial.println("Tare complete");
    }
}

void calibrate()
{
    Serial.println("***");
    Serial.println("Start calibration:");
    Serial.println("Place the load cell on a stable level surface and remove all load.");
    Serial.println("Send 't' to tare.");

    while (true)
    {
        loadCell.update();
        if (Serial.available() > 0 && Serial.read() == 't')
        {
            loadCell.tareNoDelay();
        }
        if (loadCell.getTareStatus())
        {
            Serial.println("Tare complete");
            break;
        }
    }

    Serial.println("Place a known mass on the load cell.");
    Serial.println("Send its weight, for example: 100.0");

    float knownMass = 0.0f;
    while (knownMass == 0.0f)
    {
        loadCell.update();
        if (Serial.available() > 0)
        {
            knownMass = Serial.parseFloat();
        }
    }

    Serial.print("Known mass: ");
    Serial.println(knownMass);
    loadCell.refreshDataSet();
    const float calibrationValue = loadCell.getNewCalibration(knownMass);

    Serial.print("New calibration factor: ");
    Serial.println(calibrationValue);
    Serial.println("Copy this value to CAL_FACTORS in ../../src/hx711.cpp.");
    Serial.print("Save to EEPROM address ");
    Serial.print(CAL_FACTOR_EEPROM_ADDRESS);
    Serial.println("? y/n");

    while (true)
    {
        if (Serial.available() == 0)
        {
            continue;
        }

        const char answer = static_cast<char>(Serial.read());
        if (answer == 'y')
        {
            EEPROM.begin(512);
            EEPROM.put(CAL_FACTOR_EEPROM_ADDRESS, calibrationValue);
            EEPROM.commit();
            Serial.println("Calibration factor saved to EEPROM.");
            break;
        }
        if (answer == 'n')
        {
            Serial.println("Calibration factor was not saved.");
            break;
        }
    }

    Serial.println("End calibration. Send 'r' to calibrate again or 'c' to edit the factor.");
    Serial.println("***");
}

void changeSavedCalFactor()
{
    Serial.print("Current factor: ");
    Serial.println(loadCell.getCalFactor());
    Serial.println("Send the new calibration factor.");

    float calibrationValue = 0.0f;
    while (calibrationValue == 0.0f)
    {
        if (Serial.available() > 0)
        {
            calibrationValue = Serial.parseFloat();
        }
    }

    loadCell.setCalFactor(calibrationValue);
    Serial.print("New factor set: ");
    Serial.println(calibrationValue);
}
