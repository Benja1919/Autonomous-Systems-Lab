// Define Pins
#define ENCODER_PINA 2
#define ENCODER_PINB 3
// encoder variables
volatile int encoderCounts = 0;
volatile float diff_in_time = 0;
int desired_location = 0;
const int maxPos = 255;
const int minPos = -255;
const float Ki = 1.0;
const float Kp = 1.0;
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
}
void loop() {
  while(Serial.available()){
    desired_location = Serial.readStringUntil('\n').toInt();
  }
    //static long myTime = millis();
    //diff_in_time = (millis()-myTime)/1000.0;
    diff_in_time = 0.01; //for 100Hz
    //myTime = millis();
    static float initial_error = 0.0;
    float summated_error = (float)(0.5*desired_location-encoderCounts);
    initial_error = initial_error + summated_error;//*diff_in_time;
    int pos = Kp*summated_error + Ki*initial_error;
    pos = constrain(pos, minPos, maxPos); 
    if (pos < 0) {
      digitalWrite(in1, LOW);
      analogWrite(in2, -pos);
    } else if (pos > 0) {
      digitalWrite(in2, LOW);
      analogWrite(in1, pos);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
  Serial.print(desired_location);
  Serial.print(" ");
  Serial.println(encoderCounts*2);
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
