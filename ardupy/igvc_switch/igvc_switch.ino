/*
  Attach button to pin 48.
 */

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(48, INPUT);
  pinMode(42, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(42, HIGH);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on pin 48:
  int switchValue = digitalRead(48);
  // print out the value you read:
  digitalWrite(13, switchValue);
  Serial.println(switchValue);
  //digitalWrite(13, 0);
  //digitalWrite(42, 0);
  delay(1);        // delay in between reads for stability
}
