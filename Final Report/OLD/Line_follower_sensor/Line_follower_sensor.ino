/* This example uses the line sensors on the Zumo 32U4 to follow
a black line on a white background, using a PID-based algorithm.
It works decently on courses with smooth, 6" radius curves and
has been tested with Zumos using 75:1 HP motors.  Modifications
might be required for it to work well on different courses or
with different motors.

This demo requires a Zumo 32U4 Front Sensor Array to be
connected, and jumpers on the front sensor array must be
installed in order to connect pin 4 to DN4 and pin 20 to DN2. */

#include <Wire.h>
#include <Zumo32U4.h>

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const uint16_t maxSpeed = 400;

Zumo32U4Buzzer buzzer;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;

bool proxLeftActive;
bool proxFrontActive;
bool proxRightActive;
Zumo32U4Motors motors;
Zumo32U4ButtonC buttonC;

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

void printReadingsToSerial()
{
  static char buffer[80];
  sprintf(buffer, "%d %d %d %d %d %d  %d %d %d  %4d %4d %4d %4d %4d\n",
    proxSensors.countsLeftWithLeftLeds(),
    proxSensors.countsLeftWithRightLeds(),
    proxSensors.countsFrontWithLeftLeds(),
    proxSensors.countsFrontWithRightLeds(),
    proxSensors.countsRightWithLeftLeds(),
    proxSensors.countsRightWithRightLeds(),
    proxLeftActive,
    proxFrontActive,
    proxRightActive,
    lineSensorValues[0],
    lineSensorValues[1],
    lineSensorValues[2],
    lineSensorValues[3],
    lineSensorValues[4]
  );
  Serial.print(buffer);
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

void setup()
{
   lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();

  /* Configuration 2:
   * - 5 line sensors (1, 2, 3, 4, 5)
   * - 1 proximity sensor (front)
   *
   * For this configuration to work, jumpers on the front sensor
   * array must be installed in order to connect pin 4 to DN4 and
   * pin 20 to DN2.  This is a good configuration for a line
   * follower or maze solver. */
  //lineSensors.initFiveSensors();
  //proxSensors.initFrontSensor();

  /* Configuration 3:
   * - 4 line sensors (1, 3, 4, 5)
   * - 2 proximity sensors (left, front)
   *
   * For this configuration to work, jumpers on the front sensor
   * array must be installed in order to connect pin 4 to DN4 and
   * pin 20 to LFT. */
  //uint8_t lineSensorPins[] = { SENSOR_DOWN1, SENSOR_DOWN3, SENSOR_DOWN4, SENSOR_DOWN5 };
  //lineSensors.init(lineSensorPins, sizeof(lineSensorPins));
  //uint8_t proxSensorPins[] = { SENSOR_LEFT, SENSOR_FRONT };
  //proxSensors.init(proxSensorPins, sizeof(proxSensorPins));

  /* Configuration 4:
   * - 4 line sensors (1, 2, 3, 5)
   * - 2 proximity sensors (front, right)
   *
   * For this configuration to work, jumpers on the front sensor
   * array must be installed in order to connect pin 4 to RGT and
   * pin 20 to DN2. */
  //uint8_t lineSensorPins[] = { SENSOR_DOWN1, SENSOR_DOWN2, SENSOR_DOWN3, SENSOR_DOWN5 };
  //lineSensors.init(lineSensorPins, sizeof(lineSensorPins));
  //uint8_t proxSensorPins[] = { SENSOR_FRONT, SENSOR_RIGHT };
  //proxSensors.init(proxSensorPins, sizeof(proxSensorPins));

  /* Configuration 5:
   * This is the same as configuration 1 except that all the pins
   * and parameters being used have been written explicitly, so
   * this is a good starting point if you want to do something
   * customized.
   *
   * If you use custom pins for your sensors, then the helpers
   * used in this example like countsRightWithLeftLeds() will no
   * longer know which sensor to use.  Instead, you should use
   * countsWithLeftLeds(sensorNumber) and
   * coutnsWithRightLeds(sensorNumber).
   *
   * In the code below, 2000 is timeout value (in milliseconds)
   * for the line sensors, and SENSOR_LEDON is the pin number for
   * the pin that controls the line sensor emitters. */
  //uint8_t lineSensorPins[] = { SENSOR_DOWN1, SENSOR_DOWN3, SENSOR_DOWN5 };
  //lineSensors.init(lineSensorPins, sizeof(lineSensorPins), 2000, SENSOR_LEDON);
  //uint8_t proxSensorPins[] = { SENSOR_LEFT, SENSOR_FRONT, SENSOR_RIGHT };
  //proxSensors.init(proxSensorPins, sizeof(proxSensorPins), SENSOR_LEDON);

  /* After setting up the proximity sensors with one of the
   * methods above, you can also customize their operation: */
  //proxSensors.setPeriod(420);
  //proxSensors.setPulseOnTimeUs(421);
  //proxSensors.setPulseOffTimeUs(578);
  //uint16_t levels[] = { 4, 15, 32, 55, 85, 120 };
  //proxSensors.setBrightnessLevels(levels, sizeof(levels)/2);

  //loadCustomCharacters();

  //calibrateLineSensors();


  // Uncomment if necessary to correct motor directions:
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  lineSensors.initFiveSensors();

  loadCustomCharacters();

  // Play a little welcome song
  buzzer.play(">g32>>c32");

  // Wait for button A to be pressed and released.
  display.clear();
  display.print(F("Press A"));
  display.gotoXY(0, 1);
  display.print(F("to calib"));
  buttonC.waitForButton();

  calibrateSensors();

  showReadings();

  // Play music and wait for it to finish before we start driving.
  display.clear();
  display.print(F("Go!"));
  buzzer.play("L16 cdegreg4");
  while(buzzer.isPlaying());
}

void loop()
{
  // Get the position of the line.  Note that we *must* provide
  // the "lineSensorValues" argument to readLine() here, even
  // though we are not interested in the individual sensor
  // readings.
  int16_t position = lineSensors.readLine(lineSensorValues);

  // Our "error" is how far we are away from the center of the
  // line, which corresponds to position 2000.
  int16_t error = position - 2000;

  // Get motor speed difference using proportional and derivative
  // PID terms (the integral term is generally not very useful
  // for line following).  Here we are using a proportional
  // constant of 1/4 and a derivative constant of 6, which should
  // work decently for many Zumo motor choices.  You probably
  // want to use trial and error to tune these constants for your
  // particular Zumo and line course.
  int16_t speedDifference = error / 4 + 6 * (error - lastError);

  lastError = error;

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

  // Constrain our motor speeds to be between 0 and maxSpeed.
  // One motor will always be turning at maxSpeed, and the other
  // will be at maxSpeed-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you
  // might want to allow the motor speed to go negative so that
  // it can spin in reverse.
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);

  motors.setSpeeds(leftSpeed, rightSpeed);
    static uint16_t lastSampleTime = 0;

  if ((uint16_t)(millis() - lastSampleTime) >= 100)
  {
    lastSampleTime = millis();

    // Send IR pulses and read the proximity sensors.
    proxSensors.read();

    // Just read the proximity sensors without sending pulses.
    proxLeftActive = proxSensors.readBasicLeft();
    proxFrontActive = proxSensors.readBasicFront();
    proxRightActive = proxSensors.readBasicRight();

    // Read the line sensors.
    lineSensors.read(lineSensorValues);

  printReadingsToSerial();
  }

}
