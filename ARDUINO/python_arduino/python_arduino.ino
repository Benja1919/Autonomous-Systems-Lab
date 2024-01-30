#include <ArduinoJson.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"
const int SENSOR_1_PIN = A0;
int speed = 0;
int in1 = 4;
int in2 = 5;
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
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  // Serial.print("Distance(mm): ");
//  Serial.println(distance);
  // Serial.print(",");
  if (distance >= 1500){
    speed = 255;
  }
  else{
    speed = map(distance, 0, 1500 , 127, 255);
  }
  analogWrite(in1, speed);
  analogWrite(in2 , 255- speed);
 // Serial.println(speed);
  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;
  // Serial.print("\tDistance(ft): ");
  // Serial.print(distanceFeet, 2);
 // Serial.println();
   static unsigned long last_update_time = 0;


  unsigned long current_time = millis();

  if (current_time - last_update_time >= UPDATE_INTERVAL_MS) {

    last_update_time = current_time;


    int sensor1_value = distance;


    StaticJsonDocument<64> doc;

    doc["sensor1"] = sensor1_value;

    serializeJson(doc, Serial);

    Serial.println();

  }


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
