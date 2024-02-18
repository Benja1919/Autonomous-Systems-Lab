/* Add Doc */

//#include <Wire.h>
//#include <Zumo32U4.h>
//#include "ZumoController.h"

// Constants
#define SAMPLERATE 10 // 5 millis =  200 Hz

// Desired path in the format: {x, y} coordinates
float desired_path[][2] = {
 {0.1,0.1},{0.5,0.1},{0.7,1.2},{0.1,1.67}
 }; 
int num_points = sizeof(desired_path) / sizeof(desired_path[0]);

// Zumo controller
ZumoController zumoController;

// time variables
unsigned long lastMillis = 0;
unsigned long lastMicros = 0;
float car_state_prev_x=0;
float car_state_prev_y=0;
int16_t error=0;



// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const uint16_t maxSpeed = 255;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonC buttonC;
Zumo32U4ProximitySensors proxSensors;

bool proxLeftActive;
bool proxFrontActive;
bool proxRightActive;


// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
// Zumo32U4LCD display;
Zumo32U4OLED display;

int16_t lastError = 0;

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

// Sets up special characters for the display so that we can show
// bar graphs.
void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  display.loadCustomCharacter(levels + 0, 0);  // 1 bar
  display.loadCustomCharacter(levels + 1, 1);  // 2 bars
  display.loadCustomCharacter(levels + 2, 2);  // 3 bars
  display.loadCustomCharacter(levels + 3, 3);  // 4 bars
  display.loadCustomCharacter(levels + 4, 4);  // 5 bars
  display.loadCustomCharacter(levels + 5, 5);  // 6 bars
  display.loadCustomCharacter(levels + 6, 6);  // 7 bars
}

void printBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, (char)255};
  display.print(barChars[height]);
}

void calibrateSensors()
{
  display.clear();

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-200, 200);
    }
    else
    {
      motors.setSpeeds(200, -200);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

// Shows a bar graph of sensor readings on the display.
// Returns after the user presses A.
void showReadings()
{
  display.clear();

  while(!buttonC.getSingleDebouncedPress())
  {
    lineSensors.readCalibrated(lineSensorValues);

    display.gotoXY(0, 0);
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
      uint8_t barHeight = map(lineSensorValues[i], 0, 1000, 0, 8);
      printBar(barHeight);
    }
  }
}

void line_follower_loop(void){
    int16_t error = position - 2000;
    int16_t speedDifference = error / 2 + 2 * (error - lastError);
    lastError = error;
    int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
    int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;
    leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
    motors.setSpeeds(leftSpeed, rightSpeed);
}

void calibrate_between_stages(void){
    motors.setSpeeds(200, 200);
    delay(1000);
    motors.setSpeeds(0, 0);
    zumoController.reset();
}


void path_cntrl_loop(void){
//    reset variables
//    zumoController.motorsSetSpeed(0, 0);
//    zumoController.reset();
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
    }
}

void location_calc(void){
  unsigned long dtMicros = micros()-lastMicros;
  lastMicros = micros();
  float delta_x = zumoController.car_state.posx - car_state_prev_x;
  float delta_y = zumoController.car_state.posy - car_state_prev_y;
  Serial.print(zumoController.car_state.posx);
  Serial.print(" , ");
  Serial.print(zumoController.car_state.posy);
  Serial.print(" , ");
  Serial.print(error);
  Serial.print(" , ");
  Serial.println(sqrt(sq(delta_x)+sq(delta_y))/(dtMicros / 1000000.0f));
  car_state_prev_x = zumoController.car_state.posx;
  car_state_prev_y = zumoController.car_state.posy;
}

void setup()
{
  lineSensors.initFiveSensors();
  proxSensors.initThreeSensors();
  loadCustomCharacters();
  // Play a little welcome song
  buzzer.play(">g32>>c32");
  // Wait for button C to be pressed and released.
  display.clear();
  display.print(F("Press A"));
  display.gotoXY(0, 1);
  display.print(F("to calib"));
  buttonC.waitForButton();
  calibrateSensors();
  showReadings();
  // take time stamp
  lastMillis = millis();
  lastMicros = micros();
  // calculate gyro offset
  //zumoController.gyroOffset();
  // initial conditions
  zumoController.car_state.posx = 0.042426;
  zumoController.car_state.posy = 0.042426;
  zumoController.car_state.theta = -0.78539;
  // Play music and wait for it to finish before we start driving.
  display.clear();
  display.print(F("Go!"));
  buzzer.play("L16 cdegreg4");
  while(buzzer.isPlaying());
  Serial.begin(115200);
}
void loop()
{
  int16_t position = lineSensors.readLine(lineSensorValues);
  // Send IR pulses and read the proximity sensors.
  proxSensors.read();
  // Just read the proximity sensors without sending pulses.
  proxLeftActive = proxSensors.countsLeftWithRightLeds(); //maybe unncesseary
  proxFrontActive = proxSensors.countsFrontWithRightLeds(); //maybe unncesseary
  proxRightActive = proxSensors.countsRightWithLeftLeds(); //maybe unncesseary
  delay(5);
  bool condition_active_forward = (lineSensorValues[1]>500 || lineSensorValues[2]>500 || lineSensorValues[3]>500);
  bool condition_not_active_forward = (lineSensorValues[1]<300 && lineSensorValues[2]<300 && lineSensorValues[3]<300);
  bool condition_not_active_left_right = (lineSensorValues[0]<300||lineSensorValues[4]<300);
  bool condition_active_left_right = (lineSensorValues[0]>500||lineSensorValues[4]>500);
  
  if (condition_active_forward && condition_not_active_left_right)|| (condition_not_active_forward && condition_active_left_right)
  {
    line_follower_loop();
  }
  else if (condition_active_forward && condition_active_left_right ) {
    calibrate_between_stages();
  }
  else if (condition_not_active_forward && condition_not_active_left_right) {
    path_cntrl_loop();
  }
  location_calc();
}
