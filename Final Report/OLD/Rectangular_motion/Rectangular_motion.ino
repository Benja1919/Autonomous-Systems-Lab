/*
  Path Control ZumoPi Platform
*/

#include "ZumoController.h"

// Constants
#define SAMPLERATE 10 // 5 millis =  200 Hz

// Desired path in the format: {x, y} coordinates
float desired_path[][2] = {
//    {0   , 0   },
//    {0.1   , 0.5 },
//    {-0.06  , 0.5 },
//    {-0.06  , 0   } // compensate stopping distance



 //{-0.099833,-0.0049958} , {-0.19867,-0.019933} , {-0.29552,-0.044664} , {-0.38942,-0.078939} , {-0.47943,-0.12242} , {-0.56464,-0.17466} , {-0.64422,-0.23516} , {-0.71736,-0.30329} , {-0.78333,-0.37839} , {-0.84147,-0.4597} , {-0.89121,-0.5464} , {-0.93204,-0.63764} , {-0.96356,-0.7325} , {-0.98545,-0.83003} , {-0.99749,-0.92926} , {-0.99957,-1.0292} , {-0.99166,-1.1288} , {-0.97385,-1.2272} , {-0.9463,-1.3233} , {-0.9093,-1.4161} , {-0.86321,-1.5048} , {-0.8085,-1.5885} , {-0.74571,-1.6663} , {-0.67546,-1.7374} , {-0.59847,-1.8011} , {-0.5155,-1.8569} , {-0.42738,-1.9041} , {-0.33499,-1.9422} , {-0.23925,-1.971} , {-0.14112,-1.99} , {-0.041581,-1.9991}
 //{-0.049917,-0.0024979} , {-0.099335,-0.0099667} , {-0.14776,-0.022332} , {-0.19471,-0.03947} , {-0.23971,-0.061209} , {-0.28232,-0.087332} , {-0.32211,-0.11758} , {-0.35868,-0.15165} , {-0.39166,-0.1892} , {-0.42074,-0.22985} , {-0.4456,-0.2732} , {-0.46602,-0.31882} , {-0.48178,-0.36625} , {-0.49272,-0.41502} , {-0.49875,-0.46463} , {-0.49979,-0.5146} , {-0.49583,-0.56442} , {-0.48692,-0.6136} , {-0.47315,-0.66164} , {-0.45465,-0.70807} , {-0.4316,-0.75242} , {-0.40425,-0.79425} , {-0.37285,-0.83314} , {-0.33773,-0.8687} , {-0.29924,-0.90057} , {-0.25775,-0.92844} , {-0.21369,-0.95204} , {-0.16749,-0.97111} , {-0.11962,-0.98548} , {-0.07056,-0.995} , {-0.02079,-0.99957}
  //{-0.049917,0.0524979} ,{-0.099335,0.0099667}, {-0.149,0.01495} , {-0.22164,0.033498} , {-0.29206,0.059204} , {-0.35957,0.091813} , {-0.42348,0.131} , {-0.48316,0.17637} , {-0.53802,0.22747} , {-0.5875,0.28379} , {-0.6311,0.34477} , {-0.66841,0.4098} , {-0.69903,0.47823} , {-0.72267,0.54938} , {-0.73909,0.62252} , {-0.74812,0.69695} , {-0.74968,0.7719} , {-0.74375,0.84663} , {-0.73039,0.9204} , {-0.70973,0.99247} , {-0.68197,1.0621} , {-0.64741,1.1286} , {-0.60637,1.1914} , {-0.55928,1.2497} , {-0.5066,1.303} , {-0.44885,1.3509} , {-0.38663,1.3927} , {-0.32053,1.4281} , {-0.25124,1.4567} , {-0.17944,1.4782} , {-0.10584,1.4925} , {-0.031185,1.4994}
//{-0.079867,-0.0039967} , {-0.15894,-0.015947} , {-0.23642,-0.035731} , {-0.31153,-0.063151} , {-0.38354,-0.097934} , {-0.45171,-0.13973} , {-0.51537,-0.18813} , {-0.57388,-0.24263} , {-0.62666,-0.30271} , {-0.67318,-0.36776} , {-0.71297,-0.43712} , {-0.74563,-0.51011} , {-0.77085,-0.586} , {-0.78836,-0.66403} , {-0.798,-0.74341} , {-0.79966,-0.82336} , {-0.79333,-0.90308} , {-0.77908,-0.98176} , {-0.75704,-1.0586} , {-0.72744,-1.1329} , {-0.69057,-1.2039} , {-0.6468,-1.2708} , {-0.59656,-1.333} , {-0.54037,-1.3899} , {-0.47878,-1.4409} , {-0.4124,-1.4855} , {-0.3419,-1.5233} , {-0.26799,-1.5538} , {-0.1914,-1.5768} , {-0.1129,-1.592} , {-0.033265,-1.5993} //0.8
  //{-0.0599,-0.0029975} , {-0.1192,-0.01196} , {-0.17731,-0.026798} , {-0.23365,-0.047363} , {-0.28766,-0.07345} , {-0.33879,-0.1048} , {-0.38653,-0.14109} , {-0.43041,-0.18198} , {-0.47,-0.22703} , {-0.50488,-0.27582} , {-0.53472,-0.32784} , {-0.55922,-0.38259} , {-0.57813,-0.4395} , {-0.59127,-0.49802} , {-0.5985,-0.55756} , {-0.59974,-0.61752} , {-0.595,-0.67731} , {-0.58431,-0.73632} , {-0.56778,-0.79397} , {-0.54558,-0.84969} , {-0.51793,-0.90291} , {-0.4851,-0.9531} , {-0.44742,-0.99977} , {-0.40528,-1.0424} , {-0.35908,-1.0807} , {-0.3093,-1.1141} , {-0.25643,-1.1424} , {-0.20099,-1.1653} , {-0.14355,-1.1826} , {-0.084672,-1.194} , {-0.024948,-1.1995}// 0.6
  //{-0.0029975,0.0599} , {-0.01196,0.1192} , {-0.026798,0.17731} , {-0.047363,0.23365} , {-0.07345,0.28766} , {-0.1048,0.33879} , {-0.14109,0.38653} , {-0.18198,0.43041} , {-0.22703,0.47} , {-0.27582,0.50488} , {-0.32784,0.53472} , {-0.38259,0.55922} , {-0.4395,0.57813} , {-0.49802,0.59127} , {-0.55756,0.5985} , {-0.61752,0.59974} , {-0.67731,0.595} , {-0.73632,0.58431} , {-0.79397,0.56778} , {-0.84969,0.54558} , {-0.90291,0.51793} , {-0.9531,0.4851} , {-0.99977,0.44742} , {-1.0424,0.40528} , {-1.0807,0.35908} , {-1.1141,0.3093} , {-1.1424,0.25643} , {-1.1653,0.20099} , {-1.1826,0.14355} , {-1.194,0.084672} , {-1.1995,0.024948}// up
  //{1 , 0.1} , {0.69671,0.71736}  , {-0.73739,0.67546}, {-1, 0.1}
  {0.1, 0.1}, {0.5, 0.1} , {0.5,1} , {0.01,1.2},
    // ...add more points as needed...
};

// Number of points in the path
int num_points = sizeof(desired_path) / sizeof(desired_path[0]);

// Zumo controller
ZumoController zumoController;
bool pathControlFlag = 0;
// time variables
unsigned long lastMillis = 0;
unsigned long lastMicros = 0;

// message variables
String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

// battery level;
float batteryVoltage = 0;

// add button to init path control
Zumo32U4ButtonC buttonC;

void setup() {
  // initialize serial:
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  // take time stamp
  lastMillis = millis();
  lastMicros = micros();
  // calculate gyro offset
  //zumoController.gyroOffset();
  // initial conditions
  zumoController.car_state.posx = 0;
  zumoController.car_state.posy = 0;
  zumoController.car_state.theta = 0;
}

void loop() {
  // check button press to init path controll
  if (buttonC.getSingleDebouncedPress()){ 
    // reset variables
    zumoController.motorsSetSpeed(0, 0);
    zumoController.reset();
    delay(2500);
    pathControlFlag = 1;
  }
  if (pathControlFlag){
    if (millis() - lastMillis >= SAMPLERATE){
      lastMillis = millis();
      // calculate dt sample
      unsigned long dtMicros = micros()-lastMicros;
      lastMicros = micros();
      
      zumoController.dt_time = dtMicros / 1000000.0f;
      // update current car states:
      zumoController.odometry();
      // run path control
      zumoController.P2P_CTRL(desired_path, num_points);
      
      // read voltage
      uint16_t batteryLevel = readBatteryMillivolts();
      batteryVoltage = float(batteryLevel)/1000.0;
      
      // send a response
      Serial.print(zumoController.car_state.posx);
      Serial.print(" , ");
      Serial.print(zumoController.car_state.posy);
      Serial.print(" , ");
      Serial.println(batteryVoltage);
    }
  }
  // check for incoming messages
  msg_handler();
}

void msg_handler(void){
  // check for incoming message
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  // check if message is complete
  if (stringComplete) {
    parse_msg();
  }
}
// parse incoming message and send a response
void parse_msg(void){
    //Serial.print(inputString);

    int ind1 = inputString.indexOf(',');  //finds location of first ,
    String str = inputString.substring(0, ind1);   //captures first data String
    int joyX = str.toInt();
    str ="";
    str = inputString.substring(ind1+1);   //captures first data String
    int joyY = str.toInt();
    
    int leftMotor = joyY + joyX;  //int(float(joyX)/1.5);
    int rightMotor = joyY - joyX; //int(float(joyX)/1.5);
    uint16_t batteryLevel = readBatteryMillivolts();
    batteryVoltage = float(batteryLevel)/1000.0;


    // send a response
    Serial.print(leftMotor);
    Serial.print(" , ");
    Serial.print(rightMotor);
    Serial.print(" , ");
    Serial.println(batteryVoltage);

    pathControlFlag = 0;
    zumoController.motorsSetSpeed(leftMotor,rightMotor);
    // clear the string:
    inputString = "";
    stringComplete = false;  




}
