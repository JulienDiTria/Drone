#define ledPin 13

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  
  delay(2000);
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin
  esc_setup();
  RX_setup();
  IMU_setup();
  control_loop_initPIDs();
  Serial.println("setup done");
}

void loop() {
  controlLoop_loop();
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin
}
