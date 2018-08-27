/*
Coded by Marjan Olesch
Sketch from Insctructables.com
Open source - do what you want with this code!
*/
#include <Servo.h>

int value = 0; // set values you need to zero
int speedy=0;
#define UP 180
#define MID_UP 135
#define MID 90
#define MID_DOWN 45
#define DOWN 0
#define TIME 500
#define MAPP map(speedy, 0, 180, 0  , 180)
Servo servo_motor1;

void setup() {

  servo_motor1.attach(10);    // attached to pin 9 I just do this with 1 Servo
  Serial.begin(9600);    // start serial at 9600 baud
}

int mapping_(int i){
  int angle = map(speedy, 0, 180, 500, 2500);
}

void loop() {
  if(Serial.available()){ 
    value = Serial.parseInt();    // Parse an Integer from Serial
    Serial.println(value);
    servo_motor1.write(value);
  }
  /*if(Serial.available()){
    value=Serial.read();
    if(value=='u'){
      Serial.print("write ");
      speedy=MID;
      servo_motor1.write(MAPP);
      Serial.print("MID ");
      delay(TIME);
      speedy=UP;
      servo_motor1.write(MAPP);
      Serial.print("UP ");
      delay(TIME);
      speedy=MID;
      servo_motor1.write(MAPP);
      Serial.println("MID");
      delay(TIME);
    }
    if(value=='d'){
      Serial.print("write ");
      speedy=MID;
      servo_motor1.write(MAPP);
      Serial.print("MID ");
      delay(TIME);
      speedy=DOWN;
      servo_motor1.write(MAPP);
      Serial.print("DOWN ");
      delay(TIME);
      speedy=MID;
      servo_motor1.write(MAPP);
      Serial.println("MID");
      delay(TIME);
    }
    else if(value=='0'){
      speedy=DOWN;
      servo_motor1.write(MAPP);
      Serial.println("write DOWN");
    }
    else if(value=='1'){
      speedy=MID_DOWN;
      servo_motor1.write(MAPP);
      Serial.println("write MID_DOWN");
    }
    
    else if(value=='2'){
      speedy=MID;
      servo_motor1.write(MAPP);
      Serial.println("write MID");
    }
    else if(value=='3'){
      speedy=MID_UP;
      servo_motor1.write(MAPP);
      Serial.println("write MID_UP");
    }
    else if(value=='4'){
      speedy=UP;
      servo_motor1.write(MAPP);
      Serial.println("write UP");
    }
    else if(value=='+'){
      value=servo_motor1.read();
      servo_motor1.write(value+1);
      Serial.print("write ");
      Serial.println(value+1);
    }
    else if(value=='-'){
      value=servo_motor1.read();
      servo_motor1.write(value-1);
      Serial.print("write ");
      Serial.println(value-1);
    }
  } */
}
