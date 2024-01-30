int in1 = 5;
int in2 = 6;
int speed = 127;
int led = 9;         // the PWM pin the LED is attached to
int brightness = 0; 
void setup() {
  // put your setup code here, to run once:
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(led, OUTPUT);
    Serial.begin(9600);
}
void loop() {
  // put your main code here, to run repeatedly:
  int pos = analogRead(A0);
  if (pos == 512){
    analogWrite(led, LOW);
    analogWrite(in1, HIGH);
    analogWrite(in2 , HIGH);
    Serial.println(pos);
  }
  if (pos > 512){
    speed = map(pos, 512, 1023, 127, 255);
    analogWrite(led, map(speed, 127, 255, 0, 255));
    analogWrite(in1, speed);
    analogWrite(in2 , 255 - speed);
    Serial.println(speed);
  }
  if(pos <512){
    speed = map(pos, 0, 511, 255, 127);
    analogWrite(led, map(speed, 127, 255, 0, 255));
    analogWrite(in1, 255-speed);
    analogWrite(in2 , speed);
    Serial.println(speed);

  }
}
