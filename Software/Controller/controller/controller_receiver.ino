#include "receiver.h"

//Pin Change Interrupts
#include <PinChangeInt.h>

// internal
//#define NO_PORTB_PINCHANGES //PinChangeInt setup
//#define NO_PORTC_PINCHANGES //only port D pinchanges

// internal : receiver on pin A0 -> A3
#define RX_PIN_1 A3 // pitch
#define RX_PIN_2 A2 // roll
#define RX_PIN_3 A1 // thrust
#define RX_PIN_4 A0 // yaw

// internal
volatile unsigned long RX_prev_time[4] = {0,0,0,0}; // value of time in micro sec when RX_PIN_* start to be HIGH (*=1,2,3,4)
uint8_t latest_interrupted_pin;

// external
void RX_setup(){
  // set the pins for reading interrupts
  pinMode(RX_PIN_1, INPUT); digitalWrite(RX_PIN_1, HIGH);
  pinMode(RX_PIN_2, INPUT); digitalWrite(RX_PIN_2, HIGH);
  pinMode(RX_PIN_3, INPUT); digitalWrite(RX_PIN_3, HIGH);
  pinMode(RX_PIN_4, INPUT); digitalWrite(RX_PIN_4, HIGH);

  // attach all interrupts to the pins for the receivers
  noInterrupts();           // disable all interrupts
  PCintPort::attachInterrupt(RX_PIN_1, &RX_rising, RISING);
  PCintPort::attachInterrupt(RX_PIN_2, &RX_rising, RISING);
  PCintPort::attachInterrupt(RX_PIN_3, &RX_rising, RISING);
  PCintPort::attachInterrupt(RX_PIN_4, &RX_rising, RISING);
  interrupts();             // reenable all interrupts
}

// internal
// interrupt routine for rising case of receiver 
void RX_rising()
{
  latest_interrupted_pin=PCintPort::arduinoPin; // get the latest interrupted pin
  switch(latest_interrupted_pin) // switch for that pin to know if it's an RX_PIN_*
  {
    case RX_PIN_1:
     PCintPort::attachInterrupt(RX_PIN_1, &RX_falling, FALLING); // attach a new interrupt for falling case
     RX_prev_time[RX_1] = micros(); // save the time of rising
     break;
    case RX_PIN_2:
     PCintPort::attachInterrupt(RX_PIN_2, &RX_falling, FALLING);
     RX_prev_time[RX_2] = micros();
     break;
    case RX_PIN_3:
     PCintPort::attachInterrupt(RX_PIN_3, &RX_falling, FALLING);
     RX_prev_time[RX_3] = micros();
     break;
    case RX_PIN_4:
     PCintPort::attachInterrupt(RX_PIN_4, &RX_falling, FALLING);
     RX_prev_time[RX_4] = micros();
     break;
    default:
     break;
  }
}

// internal
// interrupt routine for rising case of receiver 
void RX_falling() {
  latest_interrupted_pin=PCintPort::arduinoPin; // get the latest interrupted pin
  switch(latest_interrupted_pin) // switch for that pin to know if it's an RX_PIN_*
  {
    case RX_PIN_1:
     PCintPort::attachInterrupt(RX_PIN_1, &RX_rising, RISING); // attach a new interrupt for rising case
     RX_time_high[RX_1] = micros()-RX_prev_time[RX_1]; // save the time of high
     break;
    case RX_PIN_2:
     PCintPort::attachInterrupt(RX_PIN_2, &RX_rising, RISING);
     RX_time_high[RX_2] = micros()-RX_prev_time[RX_2];
     break;
    case RX_PIN_3:
     PCintPort::attachInterrupt(RX_PIN_3, &RX_rising, RISING);
     RX_time_high[RX_3] = micros()-RX_prev_time[RX_3];
     break;
    case RX_PIN_4:
     PCintPort::attachInterrupt(RX_PIN_4, &RX_rising, RISING);
     RX_time_high[RX_4] = micros()-RX_prev_time[RX_4];
     break;
    default:
     break;
  }
}

