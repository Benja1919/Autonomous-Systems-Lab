//Encoder Example
// Define Pins
#define ENCODER_PINA 2
#define ENCODER_PINB 3
// encoder variables
volatile int encoderCounts = 0;
volatile int num_of_rotations = 0;
volatile float diff = 0;
unsigned long myTime = 0;
volatile long last_my_time = 0;

float RPM = 0.0;
int in1 = 5;
int in2 = 6;
int speed = 127;
int led = 9;         // the PWM pin the LED is attached to
int brightness = 0; 
// Encoder ISR functions - Interupt Service Routine
void encoderA();
void encoderB();
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin (115200);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(led, OUTPUT);
  // initialize encoder, attache ISR functions
  pinMode(ENCODER_PINA, INPUT);
  pinMode(ENCODER_PINB, INPUT);
   // Attached interrupt to encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINB), encoderB, CHANGE);
}
void loop() {
  int pos = analogRead(A0);
  if (pos == 512){
    analogWrite(led, LOW);
    analogWrite(in1, HIGH);
    analogWrite(in2 , HIGH);
  }
  if (pos > 512){
    speed = map(pos, 512, 1023, 127, 255);
    analogWrite(led, map(speed, 127, 255, 0, 255));
    analogWrite(in1, speed);
    analogWrite(in2 , 255 - speed);
  }
  if(pos <512){
    speed = map(pos, 0, 511, 255, 127);
    analogWrite(led, map(speed, 127, 255, 0, 255));
    analogWrite(in1, 255-speed);
    analogWrite(in2 , speed);
  }
  if (abs(encoderCounts) / 618 == 1)
  {
    myTime = millis();
    //Serial.println("RPM: ");
    RPM = 60000/((myTime-last_my_time));
    //Serial.println(myTime-last_my_time);
    
    last_my_time = myTime;
    Serial.println(RPM);
   // Serial.println("Num_of_Rotations: ");
    num_of_rotations = num_of_rotations + 1;
    
   // Serial.println(num_of_rotations);
    encoderCounts = 0;
    
  }
  // print encoder position
  delay(10);
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
