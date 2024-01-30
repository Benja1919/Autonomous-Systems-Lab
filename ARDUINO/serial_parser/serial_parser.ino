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
  while(Serial.available() > 0) {
    analogWrite(red, Serial.readStringUntil(',').toInt());
    analogWrite(green, Serial.readStringUntil(',').toInt());
    analogWrite(blue, Serial.readStringUntil('\n').toInt());
  }

}
