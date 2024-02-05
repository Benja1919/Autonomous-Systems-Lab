/*
  The following code implements the close loop velocity with PID controller. we have a similar implementation to the one with velocity, only this time we took the delta of positions 
  and by dividing it by the delta of time we manage to control the speed. we used our implementations in the previous sections regarding the motor, RPM and speed to reach this result.
  In addition, several variables of the controller and filtering were chose based on different measurements and results (several to see which is effective) as described in the code.
  This time the user needs to input a string to determine the desired speed.
*/
// Define Pins
#define ENCODER_PINA 2
#define ENCODER_PINB 3
// encoder variables
volatile int encoderCounts = 0;
//inintiazlize different variables
volatile double diff_in_time = 0;
double desired_speed = 0.0;
const double maxLim = 200.0;
const double minLim = -200.0;
double lastTime = 0.0;
double curr_loc = 0.0;
double prev_loc = 0.0;
double prev_speed = 0.0;
//for PID controller
const double Ki = 5.0; //we chose 5.0 after several attempts and got decent result
const double Kp = 1.0;
//for filtering
double Alpha = 0.1; //we chose 0.1 after several attempts and got decent result
int in1 = 5;
int in2 = 6;
// Encoder ISR functions - Interupt Service Routine
void encoderA();
void encoderB();

void setup() {
  Serial.begin (115200);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // initialize encoder, attache ISR functions
  pinMode(ENCODER_PINA, INPUT);
  pinMode(ENCODER_PINB, INPUT);
   // Attached interrupt to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINB), encoderB, CHANGE);
  delay(1);
}
void loop() {
  //This part is for the input from the user
  while(Serial.available()){
    desired_speed = Serial.readStringUntil('\n').toDouble();
  }
    diff_in_time = 0.01; //for 100Hz
    double myTime = millis();
    double dt = (myTime - lastTime)/1000.0;  
    curr_loc = (double) encoderCounts/(12*50); //num of ticks divided by counts per rev multiplied by our gear-ratio  
    double dt_pos = curr_loc - prev_loc;  
    double curr_speed = (dt_pos / dt)*60; //diff in pos divided by diff in time mulitplied by 60 for minutes  
    curr_speed = (Alpha * curr_speed + (1.0 - Alpha) * prev_speed); //filtering with alpha  
    //This part is for the PI controller
    static double initial_error = 0.0; //will be updated in the iterations
    double summated_error = (desired_speed-curr_speed);   
    initial_error = initial_error + summated_error*diff_in_time;
    double RPM = Kp*summated_error + Ki*initial_error; //Controller equation
    RPM = constrain(RPM, minLim, maxLim); //for the limits declared at top
    if (RPM < 0) {
      digitalWrite(in1, LOW);
      analogWrite(in2, -RPM);
    } else if (RPM > 0) {
      digitalWrite(in2, LOW);
      analogWrite(in1, RPM);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
  //plotting the desired next to the actual location
    Serial.print(curr_speed);
    Serial.print(" ");
    Serial.println(desired_speed);
    lastTime = myTime;
    prev_loc = curr_loc;
    prev_speed = curr_speed;
  delay(10); //for 100Hz
  }
// EncoderA ISR
void encoderA() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_PINA) == HIGH) {
    // check channel A to see which way encoder is turning
    digitalRead(ENCODER_PINB) ? encoderCounts++ : encoderCounts--;    
  }else{
    // check channel A to see which way encoder is turning
    digitalRead(ENCODER_PINB) ? encoderCounts-- : encoderCounts++; 
  } 
} // End EncoderA ISR
// EncoderB ISR
void encoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_PINB) == HIGH) {
    // check channel A to see which way encoder is turning
    digitalRead(ENCODER_PINA) ? encoderCounts-- : encoderCounts++;    
  }else{
    // check channel A to see which way encoder is turning
    digitalRead(ENCODER_PINA) ? encoderCounts++ : encoderCounts--; 
  }
} // End EncoderB ISR
