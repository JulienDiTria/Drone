#ifndef RX_H
#define RX_H

#define RX_1 0 // index for memory for input num * from receiver
#define RX_2 1 // (*=1,2,3,4)
#define RX_3 2
#define RX_4 3

#define RX_MIN 1000 // us
#define RX_MAX 2000 // us

volatile unsigned long RX_time_high[4] = {0,0,0,0}; // value of time in micros sec during wich RX_PIN_* is HIGH (*=1,2,3,4)

#endif
