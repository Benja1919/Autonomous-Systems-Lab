/*
  The following code combines the Arduino example code with the distance sensor
  in order to do so, we included the libraries we used in the distance_sensor part
  and also defined the sensor to be "distance" variable as declared in this section
*/
#include <ArduinoJson.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
const int UPDATE_INTERVAL_MS = 10;
SFEVL53L1X distanceSensor;
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;     // whether the string is complete
void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");
  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  while (!Serial) continue; // Wait for serial connection
  inputString.reserve(200);
}
void loop() {
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  //in this section we'll define the distance variable 
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  int sensor1_value = distance;
  StaticJsonDocument<64> doc;
  doc["sensor1"] = sensor1_value;
  serializeJson(doc, Serial);
  Serial.println();
  processIncomingString();
}
void processIncomingString() {
  if (stringComplete) {
  StaticJsonDocument<64> incoming_doc;
  DeserializationError error = deserializeJson(incoming_doc, inputString);
  if (!error) {
    if (incoming_doc.containsKey("command")) {
        String command = incoming_doc["command"].as<String>();
        if (command == "reset") {
          Serial.println("{'reset': 0}");
        }
      }
    }
    inputString = "";
    stringComplete = false;
  }
}
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
