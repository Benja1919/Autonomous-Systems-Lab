/*
  The following code gets a input from the user containing 3 strings regarding each pin's brightness.
  we defined the delimeter as a ',' but it can be changed to a different one. 
*/

//definition of colors to each pin
const int red = 9;
const int green = 10;
const int blue = 11;

void setup() {
  Serial.begin(9600);
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
}

void loop() {
  while(Serial.available() > 0) { //as long as the user can send the input string
    analogWrite(red, Serial.readStringUntil(',').toInt());
    analogWrite(green, Serial.readStringUntil(',').toInt());
    analogWrite(blue, Serial.readStringUntil('\n').toInt());
  }

}
