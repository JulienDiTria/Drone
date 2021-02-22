#include <Servo.h> 
#include "esc.h"

// internal
Servo esc_motor1, esc_motor2, esc_motor3, esc_motor4;

//internal
#define ESC_CHARGING 1000
#define ESC_CHARGED 1500

// internal : motor on pins 6,9,10,11
#define ESC_PIN_MOT1 9  // bleu
#define ESC_PIN_MOT3 10 // jaune
#define ESC_PIN_MOT2 11 // orange
#define ESC_PIN_MOT4 6  // vert

// external
void esc_setup(){

  // attaching the escs on pin ESC_PIN_MOT* to the servo objects
  esc_motor1.attach(ESC_PIN_MOT1);  
  esc_motor2.attach(ESC_PIN_MOT2);
  esc_motor3.attach(ESC_PIN_MOT3);
  esc_motor4.attach(ESC_PIN_MOT4);
  
  // charging the escs 
  esc_motor1.write(ESC_CHARGING);
  esc_motor2.write(ESC_CHARGING);
  esc_motor3.write(ESC_CHARGING);
  esc_motor4.write(ESC_CHARGING);
  
  Serial.println("esc_setup done");
}

// external
void esc_write_all(){
  esc_motor1.write(constrain(esc1, ESC_MIN, ESC_MAX));
  esc_motor2.write(constrain(esc2, ESC_MIN, ESC_MAX));
  esc_motor3.write(constrain(esc3, ESC_MIN, ESC_MAX));
  esc_motor4.write(constrain(esc4, ESC_MIN, ESC_MAX));
}

