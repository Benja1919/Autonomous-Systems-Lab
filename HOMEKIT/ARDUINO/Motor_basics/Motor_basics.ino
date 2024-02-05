/* This code, spin the motor CW for 5 seconde, break for 5 seconds, spin CCW for 5 seconds, break and so on. */
  
  int in1 = 5;
int in2 = 6;
void setup() {
  // Set the 'output' pins
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
}
void loop() {
  // Spin CW for 5 seconds
  digitalWrite (in1, HIGH);
  digitalWrite (in2, LOW);
  delay(5000);
  //Break for 5 seconds
  digitalWrite (in1, HIGH);
  digitalWrite (in2, HIGH);
  delay(5000);
  //spin CCW for 5 seconds
  digitalWrite (in1, LOW);
  digitalWrite (in2, HIGH);
  delay(5000);
  //Break again for 5 seconds (if needed)
  digitalWrite (in1, HIGH);
  digitalWrite (in2, HIGH);
  delay(5000);
}
