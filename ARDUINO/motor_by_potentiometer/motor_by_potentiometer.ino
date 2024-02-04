/*
  This code spins the motor based on the potentiometer position.
  i.e. Middle - breaks. Right - rotates CW and Left - rotates CCW. The more off center the position the faster the rotation.
*/
  breaks while the potentiometer in right in the middle, spin CW when its right to in and
int in1 = 5;
int in2 = 6;
int speed = 127;
int led = 9;         // The PWM pin the LED is attached to
int brightness = 0; 
void setup() {
  // Set the outputs
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(led, OUTPUT);
    Serial.begin(9600);
}
void loop() {

  int pos = analogRead(A0); //Get the position from potentiometer
  if (pos == 512){ //Break
    analogWrite(led, LOW);
    analogWrite(in1, HIGH);
    analogWrite(in2 , HIGH);
    Serial.println(pos);
  }
  if (pos > 512){ //Spin to the right
    speed = map(pos, 512, 1023, 127, 255); //Calc the speed of the spin in terms of in1/2 (value between 0-255)
    analogWrite(led, map(speed, 127, 255, 0, 255)); // Set the LEDs power
    analogWrite(in1, speed);
    analogWrite(in2 , 255 - speed);
    Serial.println(speed);
  }
  if(pos <512){ // Spin to the lef
    speed = map(pos, 0, 511, 255, 127); //Calc the speed of the spin in terms of in1/2 (value between 0-255)
    analogWrite(led, map(speed, 127, 255, 0, 255)); // Set the LEDs power
    analogWrite(in1, 255-speed);
    analogWrite(in2 , speed);
    Serial.println(speed);

  }
}
