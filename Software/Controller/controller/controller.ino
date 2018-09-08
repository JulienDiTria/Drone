void setup() {
  delay(2000);
  esc_setup();
  RX_setup();
  IMU_setup();
  control_loop_initPIDs();
}

void loop() {
  controlLoop_loop();
}
