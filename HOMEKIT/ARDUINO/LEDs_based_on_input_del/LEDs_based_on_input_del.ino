/*
  This code gets an input fron the user (from serial), and adjust the LED's in accordance.
*/
// pins for the LEDs:

const int red = 9 ;
const int green = 10;
const int blue = 11;

void setup() {

  // initialize serial:

  Serial.begin(9600);

  // make the pins outputs:
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
}

void loop() {
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    analogWrite(red, Serial.readStringUntil(',').toInt());
    analogWrite(green, Serial.readStringUntil(',').toInt());
    analogWrite(blue, Serial.readStringUntil('\n').toInt());
  }
  
}
