#include <Servo.h> 

Servo esc_motor1,      esc_motor2,      esc_motor3,       esc_motor4;
int   ESC_CHARGING=900, ESC_MIN=1200,    ESC_MAX=1800;
int   ESC_MOT1_PIN=9,  ESC_MOT2_PIN=10, ESC_MOT3_PIN=11,  ESC_MOT4_PIN=6;

int   pos1=0,  pos2=0, pos3=0,  pos4=0;

bool SERIAL_ACTIVE = false;

void setup() {

  if ( SERIAL_ACTIVE ){
    // starting serial communication at 9600 bauds
    Serial.begin(9600);
  
    // Wait for input
    while (!Serial.available());
    Serial.read();

    Serial.println("charging all escs");
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // attaching the escs on pin ESC_MOT*_PIN to the servo objects
  esc_motor1.attach(ESC_MOT1_PIN, 1000, 2000);
//  esc_motor2.attach(ESC_MOT2_PIN);
//  esc_motor3.attach(ESC_MOT3_PIN);
//  esc_motor4.attach(ESC_MOT4_PIN);

  //delay(2000);
  // charging the escs
  esc_motor1.writeMicroseconds(ESC_CHARGING);
  delay(2000);
  esc_motor1.writeMicroseconds(ESC_MIN);
  delay(2000);
//  esc_motor2.write(ESC_CHARGING);
//  esc_motor3.write(ESC_CHARGING);
//  esc_motor4.write(ESC_CHARGING);

  if ( SERIAL_ACTIVE ){
    while (!Serial.available());
    Serial.read();
    
    Serial.print("all escs charged at ");
    Serial.println(ESC_CHARGING);
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // testing each esc one at a time
  if ( SERIAL_ACTIVE ){
    Serial.println("testing each esc one at a time");
    Serial.println("esc nb 1");
  }
  for(pos1 = ESC_MIN; pos1 <= ESC_MAX; pos1 += 5) 
  {                                  
    esc_motor1.writeMicroseconds(pos1);
    if ( SERIAL_ACTIVE ){
      Serial.println(pos1);
    }
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
    delay(90);                      
  } 
  esc_motor1.writeMicroseconds(ESC_MIN);
  
//  Serial.println("esc nb 2");
//  for(pos2 = ESC_MIN; pos2 <= ESC_MAX; pos2 += 1) 
//  {                                 
//    esc_motor2.write(pos2);               
//    Serial.println(pos2);
//    delay(1000);                      
//  } 
//  esc_motor2.write(ESC_MIN);
//  
//  Serial.println("esc nb 3");
//  for(pos3 = ESC_MIN; pos3 <= ESC_MAX; pos3 += 1) 
//  {                                  
//    esc_motor3.write(pos3);               
//    Serial.println(pos3);
//    delay(1000);                      
//  } 
//  esc_motor3.write(ESC_MIN);
//
//  Serial.println("esc nb 4");
//  for(pos4 = ESC_MIN; pos4 <= ESC_MAX; pos4 += 1) 
//  {                                  
//    esc_motor4.write(pos4);               
//    Serial.println(pos4);
//    delay(1000);                      
//  } 
//  esc_motor4.write(ESC_MIN);
//  Serial.println("all testing done for all each esc");
//
//  // testing all escs at the same time
//  Serial.println("testing all escs at the same time");
//  for(pos1 = ESC_MIN; pos1 <= ESC_MAX; pos1 += 1) 
//  {                                  
//    esc_motor1.write(pos1);
//    esc_motor2.write(pos1); 
//    esc_motor3.write(pos1); 
//    esc_motor4.write(pos1);           
//    Serial.println(pos1);
//    delay(1000);                      
//  } 
//  esc_motor1.write(ESC_MIN);
//  esc_motor2.write(ESC_MIN);
//  esc_motor3.write(ESC_MIN);
//  esc_motor4.write(ESC_MIN);
}
