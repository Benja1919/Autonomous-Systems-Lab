#include <Wire.h>
#include "SparkFun_VL53L1X.h"
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3
#define TONE_PIN 11
SFEVL53L1X distanceSensor;
void setup(void)
{
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("START " __FILE__));
#if !defined(ESP32) && !defined(ARDUINO_SAM_DUE) && !defined(__SAM3X8E__)
  tone(TONE_PIN, 1000, 100);
  delay(200);
#endif
  if (distanceSensor.begin() != 0)
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1);
  }
  Serial.println("Sensor online!");
  distanceSensor.setDistanceModeShort();
  distanceSensor.setTimingBudgetInMs(50);
  distanceSensor.setIntermeasurementPeriod(100);
  distanceSensor.startRanging();
}
void loop(void)
{
  if (distanceSensor.checkForDataReady())
  {
    byte rangeStatus = distanceSensor.getRangeStatus();
    unsigned int distance = distanceSensor.getDistance();
    distanceSensor.clearInterrupt();
    unsigned int tSignalRate = distanceSensor.getSignalRate();
    unsigned int tAmbientRate = distanceSensor.getAmbientRate();
    if (rangeStatus == 0)
    {
#if !defined(ESP32) && !defined(ARDUINO_SAM_DUE) && !defined(__SAM3X8E__)
      tone(11, distance + 500);
#endif
    }
    else
    {
      distance = rangeStatus;
#if !defined(ESP32) && !defined(ARDUINO_SAM_DUE) && !defined(__SAM3X8E__)
      noTone(11);
#endif
    }
    Serial.print(distance);
    Serial.print(' ');
    Serial.print(tSignalRate / 100);
    Serial.print(' ');
    Serial.println(tAmbientRate / 100);
  }

  // Your non-blocking code can go here
}
