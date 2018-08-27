#include <Servo.h> 
#include "esc.h"

// internal
Servo esc_motor1, esc_motor2, esc_motor3, esc_motor4;

//internal
#define ESC_CHARGING 1500

// internal : motor on pins 6,9,10,11
#define ESC_PIN_MOT1 9  // bleu
#define ESC_PIN_MOT3 10 // jaune
#define ESC_PIN_MOT2 11 // orange
#define ESC_PIN_MOT4 6  // vert

// external
void esc_setup(){
  debug_println_str("esc_setup_start");
  
  // attaching the escs on pin ESC_PIN_MOT* to the servo objects
  esc_motor1.attach(ESC_PIN_MOT1);  
  esc_motor2.attach(ESC_PIN_MOT2);
  esc_motor3.attach(ESC_PIN_MOT3);
  esc_motor4.attach(ESC_PIN_MOT4);
  
  debug_println_str("esc_setup_mid");
  
  // charging the escs 
  esc_motor1.write(ESC_CHARGING);
  esc_motor2.write(ESC_CHARGING);
  esc_motor3.write(ESC_CHARGING);
  esc_motor4.write(ESC_CHARGING);
  
  debug_println_str("esc_setup done");
}

// external
void esc_write_all(){
  esc_motor1.write(esc1);
  esc_motor2.write(esc2);
  esc_motor3.write(esc3);
  esc_motor4.write(esc4);
}

