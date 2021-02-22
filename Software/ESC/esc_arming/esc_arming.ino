#include <Servo.h> 

#define sec *1000
Servo esc_motor;
int   ESC_MIN=1000,    ESC_MAX=1800;
int   ESC_PIN=9;
int   pos=0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  esc_motor.attach(ESC_PIN);
  arming();
  delay(5 sec);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(pos = ESC_MIN; pos <= ESC_MAX; pos += 5) 
  {                                  
//    esc_motor.writeMicroseconds(pos);
    blink();                  
  }
  delay(2 sec);
  for(pos = ESC_MAX; pos >= ESC_MIN; pos -= 5) 
  {                                  
//    esc_motor.writeMicroseconds(pos);
    blink();                     
  }
  delay(2 sec);
}

void arming() {
  blink();
  delay(1 sec);
  esc_motor.writeMicroseconds(ESC_MIN);
  blink();
  delay(2 sec);
  esc_motor.writeMicroseconds(ESC_MAX);
  blink();
  delay(1 sec);
  esc_motor.writeMicroseconds(ESC_MIN);
  blink();
  delay(1 sec);
}

void blink(){
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);
  delay(90);
}
