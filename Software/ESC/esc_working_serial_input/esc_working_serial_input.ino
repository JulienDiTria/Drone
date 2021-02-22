#include <Servo.h> 

int test=0;
Servo servo_motor1;
int pos=1000;

void setup() {
  servo_motor1.attach(9);  // attaches the servo on pin 10 to the servo object
  Serial.begin(9600);
  delay(10);
  servo_motor1.write(pos);
  Serial.print("charged at ");
  Serial.println(pos);
  // Wait for input
  while (!Serial.available());
  Serial.read();
}

void loop() {
  if(test==0){
    for(pos = 1020; pos <= 1900; pos += 10) // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      servo_motor1.write(pos);              // tell servo to go to position in variable 'pos' 
      Serial.println(pos);
      delay(100);                      // waits 0.5s for the servo to reach the position 
    }
    Serial.println("test current 10s");
    pos=1250;
    servo_motor1.write(pos);              // tell servo to go to position in variable 'pos' 
    Serial.println(pos);
    delay(3000);
    Serial.println("test done, waiting for manual test");
    while (!Serial.available());
    pos = Serial.parseInt();
    Serial.println(pos);
    servo_motor1.write(pos);
    delay(2000);
    Serial.println("manual test done, waiting for restart");
    pos = 1000;
    servo_motor1.write(pos);
    while (!Serial.available());
    Serial.read();
  }
  /*
  else if(test==1){
    while(Serial.available()>0){
      pos=Serial.parseInt();
      servo_motor1.write(pos);              // tell servo to go to position in variable 'pos'
      Serial.print("received : ");
      Serial.println(pos);
    }
    servo_motor1.write(pos);
  }*/
}
