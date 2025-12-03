void setup() {
  Serial.begin(115200);
  NoU3.begin();
  NoU3.calibrateIMUs();
}

void loop() {
  Serial.println(NoU3.yaw); // print yaw continuously
  delay(50);
}
