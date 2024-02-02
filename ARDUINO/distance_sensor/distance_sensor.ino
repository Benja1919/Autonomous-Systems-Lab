#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3
int speed = 0;
int in1 = 4;
int in2 = 5;
SFEVL53L1X distanceSensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
void setup(void)
{
  Wire.begin();
  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");
  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");
}
void loop(void)
{
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  // Serial.print("Distance(mm): ");
  Serial.println(distance);
  // Serial.print(",");
  if (distance >= 1500){ //assume the the max valid length is 1.5m
    speed = 255;
  }
  else{
    speed = map(distance, 0, 1500 , 127, 255); //Set speed proportionally to distance.
  }
  analogWrite(in1, speed);
  analogWrite(in2 , 255- speed);
  Serial.println(speed);
  //float distanceInches = distance * 0.0393701;
  //float distanceFeet = distanceInches / 12.0;
  // Serial.print("\tDistance(ft): ");
  // Serial.print(distanceFeet, 2);
  Serial.println();
}
