#include <Servo.h> 

Servo servo_motor1, servo_motor2, servo_motor3, servo_motor4;
int pos=45;

void setup() {
  servo_motor1.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
  delay(1);
  servo_motor1.write(pos);
  //Serial.println("charged at");
  //Serial.print(pos);
  delay(1000);
}

void loop() {
  for(pos = 60; pos <= 120; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    servo_motor1.write(pos);              // tell servo to go to position in variable 'pos' 
    Serial.println(pos);
    delay(200);                      // waits 0.2s for the servo to reach the position 
  } 
}
