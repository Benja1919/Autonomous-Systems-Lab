/***************Ben Cohen 208685784***************/
/*************Ori Avrahami 315416057*************/

/* The following code implements the final project of 2023-2024 SemesterA
we got a path in which we needed to both follow a black line and also 
use path control in order to complete a full round of motion.
The following code implemets thus by using the information learned in 
class during the seminars. In this code we use both line_followe code
provided in class, and also the code for sensors shown in seminar and 
the code for path control. All three were merged (with adjusments) so 
that the car can move autonomously according to the desire*/

/****************Include Libraries****************/
#include <Wire.h>
#include <Zumo32U4.h>
#include "ZumoController.h"
/************************************************/

/****************Variables***********************/
#define SAMPLERATE 10 // 5 millis =  200 Hz
#define NUM_SENSORS 5

//points variables
// Desired path in the format: {x, y} coordinates
float desired_path[][2] = {
 //{0.1,0.1},{0.5,0.1},{0.7,1.2},{0.1,1.67} //for vehicle 116
  {0.1, 0.1}, {0.3, 0.1} , {0.3,1.1} , {-0.4,1.18} //for vehicle 115
 };  
int num_points = sizeof(desired_path) / sizeof(desired_path[0]);


// time variables
unsigned long lastMillis = 0;
unsigned long lastMicros = 0;
float car_state_prev_x=0;
float car_state_prev_y=0;
int16_t error=0;
int16_t lastError = 0;

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const uint16_t maxSpeed = 255; //we checked and limited the car's max-speed to 255 due to our tests

//Zumo Variables
ZumoController zumoController;
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
Zumo32U4LCD display;
//DZumo32U4OLED display;

//for sensors
unsigned int lineSensorValues[NUM_SENSORS];
/************************************************/


/****************Functions***********************/

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

// prints bar at display in comparison to height
void printBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, (char)255};
  display.print(barChars[height]);
}

// function for calibrating the sensors - and adjusting speed accordingly
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

//the function that executes the line_follower loop and will execute when 
//the condition for line_follower are matched
void line_follower_loop(void){
    int16_t position = lineSensors.readLine(lineSensorValues);
    int16_t error = position - 2000;
    int16_t speedDifference = error / 2 + 1 * (error - lastError);
    lastError = error;
    int16_t leftSpeed = (int16_t)maxSpeed + speedDifference; //adjust speed for left_motor
    int16_t rightSpeed = (int16_t)maxSpeed - speedDifference; //adjust speed for right_motor
    leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed); //adjust speed according to max
    rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed); //adjust speed according to max
    motors.setSpeeds(leftSpeed, rightSpeed); //set speed
 //   zumoController.odometry();
}

//the function that executes the stage between line_follower and the path_control
//will execute when the condition for this calibration is matched
void calibrate_between_stages(void){
    int fixed_speed = 200; //a fixed size of speed that will get the car out of line before executing path_control
    motors.setSpeeds(fixed_speed, fixed_speed); //set given speed 
    int delay_before_path = 1000; //delay until we will get out of lines and will execute path_control
    delay(delay_before_path); 
    motors.setSpeeds(0, 0); //set speed to zero before executing path_control
    zumoController.reset();
}

//the function that executes the path_control loop and will execute when 
//the condition for path_control are matched
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

void result_printer(void){
  unsigned long dtMicros = micros()-lastMicros;
  lastMicros = micros();
  //calculate delta of x and y according to prev and curr locations
  float delta_x = zumoController.car_state.posx - car_state_prev_x; 
  float delta_y = zumoController.car_state.posy - car_state_prev_y;
  float curr_speed = sqrt(sq(delta_x)+sq(delta_y))/(dtMicros / 1000000.0f);
  //print current locations (x,y), error, time, speed
  Serial.print(zumoController.car_state.posx);
  Serial.print(" , ");
  Serial.print(zumoController.car_state.posy);
  Serial.print(" , ");
  Serial.print(error);
  Serial.print(" , ");
  Serial.print(lastMicros);
  Serial.print(" , ");
  Serial.println(curr_speed); //calc for speed
  //save curr values as prev in the end of func
  car_state_prev_x = zumoController.car_state.posx;
  car_state_prev_y = zumoController.car_state.posy;
}

/************************************************/

/****************Setup***************************/

//setup for the program that includes the necessary setup for both the line_follower code
//and the sensors code together
void setup()
{
  lineSensors.initFiveSensors();
  proxSensors.initThreeSensors();
  loadCustomCharacters();
  // Play a little welcome song
  buzzer.play(">g32>>c32");
  // Wait for button C to be pressed and released.
  display.clear();
  display.print(F("Press C"));
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
  // Play music and wait for it to finish before we start driving.
  display.clear();
  display.print(F("Go!"));
  buzzer.play("L16 cdegreg4");
  while(buzzer.isPlaying());
  //for baud
  Serial.begin(115200);
}

/************************************************/

/****************Loop****************************/
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
  //set the following conditions according to the sensors levels
  //categorize the conditions to active/not active, and forward/sides
  bool condition_active_forward = (lineSensorValues[1]>500 || lineSensorValues[2]>500 || lineSensorValues[3]>500);
  bool condition_not_active_forward = (lineSensorValues[1]<300 && lineSensorValues[2]<300 && lineSensorValues[3]<300);
  bool condition_not_active_left_right = (lineSensorValues[0]<300||lineSensorValues[4]<300);
  bool condition_active_left_right = (lineSensorValues[0]>500||lineSensorValues[4]>500);
  
  if ((condition_active_forward && condition_not_active_left_right)|| (condition_not_active_forward && condition_active_left_right))
  //if we see a line in front and no line in sides(or the opposite)->line_follower loop
  {
    line_follower_loop();
  }
  else if (condition_active_forward && condition_active_left_right ) {
  //if we see a line in front and also line in sides->calibrate_between_stages of line_follower and path_control
    calibrate_between_stages();
  }
  else if (condition_not_active_forward && condition_not_active_left_right) {
  //if we don't see a line at all ->path_control loop
    path_cntrl_loop();
  }
  //at the end of all conditions -> print data according to result_printer()
  result_printer();
}
/************************************************/
