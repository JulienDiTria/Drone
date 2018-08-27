
bool debug = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  esc_setup();
  RX_setup();
  IMU_setup();
  Serial.println("all setup done, waiting input to start");
  while (!Serial.available());
  Serial.read();
}

unsigned long t0, t1, t3=0;
int count=0;

void loop() {
  
  if(count == 0){
    t0 = micros();  
  }
  count++;
  if(count == 10){
    count = 0;
    t1 = micros() - t0;
    Serial.print("time for main loop in us : ");
    Serial.println(t1/10);
  }
  
  controlLoop_loop();
}


void debug_println_int(int toPrint){
  if(debug){
    Serial.println(toPrint);
    delay(100);
  }
}

void debug_print_int(int toPrint){
  if(debug){
    Serial.print(toPrint);
    delay(100);
  }
}

void debug_println_str(String toPrint){
  if(debug){
    Serial.println(toPrint);
    delay(100);
  }
}

void debug_print_str(String toPrint){
  if(debug){
    Serial.print(toPrint);
    delay(100);
  }
}

