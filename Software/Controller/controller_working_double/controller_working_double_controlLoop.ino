#include <PID_vDITRIA.h> // pid library
#include <math.h>   // math library
#include "esc.h"
#include "imu.h"
#include "receiver.h"

// internal
#define DEG_TO_RAD (M_PI/180.)
#define RAD_TO_DEG (180./M_PI)

// internal
#define K_thrust 100
#define K 1

#define ANGLE_MIN (-10*DEG_TO_RAD) // corresponds to -45 deg
#define ANGLE_MAX (10*DEG_TO_RAD)  // corresponds to  45 deg
#define RATE_MIN (-5*DEG_TO_RAD)// corresponds to -4.5 deg/s
#define RATE_MAX (5*DEG_TO_RAD)  // corresponds to  4.5 deg/s
#define THRUST_MIN 0 // corresponds to 0 cm/s
#define THRUST_MAX 100  // corresponds to 100 cm/s
#define W_MIN 1000 // rad/s
#define W_MAX 10000 // rad/s

// internal
double roll_des, pitch_des; // rad
double roll_rate_des, pitch_rate_des, yaw_rate_des; // rad/s
double throttle_des; // sqrt(m/s)

double roll_meas, pitch_meas; // rad
double roll_rate_meas, pitch_rate_meas, yaw_rate_meas; // rad/s

// internal
double roll_rate_control, pitch_rate_control, yaw_rate_control; // rad/s
double w1, w2, w3, w4; // rad/s

// internal
double Kp=2, Ki=0, Kd=0;
PID roll_rate_pid(&roll_rate_meas, &roll_rate_control, &roll_rate_des, Kp, Ki, Kd);
PID pitch_rate_pid(&pitch_rate_meas, &pitch_rate_control, &pitch_rate_des, Kp, Ki, Kd);
PID yaw_rate_pid(&yaw_rate_meas, &yaw_rate_control, &yaw_rate_des, Kp, Ki, Kd);

PID roll_pid(&roll_meas, &roll_rate_des, &roll_des, Kp, Ki, Kd);
PID pitch_pid(&pitch_meas, &pitch_rate_des, &pitch_des, Kp, Ki, Kd);

int cnt = 10;
String controlLoop_str_debug = "";
unsigned long debug_t0[7], debug_t1[7];

// external
void controlLoop_loop(){
  debug_t0[0] = micros();
  
  // outer loop 10 times less often than inner loop
  if(cnt == 10 ){
    controlLoop_outer();
    cnt = 0;
  }
  cnt ++;

  // inner loop
  controlLoop_inner();
  controlLoop_setESCoutputs();
  esc_write_all();
  
  debug_t1[0] = micros() - debug_t0[0];
  debug_printTime();
}

// internal
// get the inputs from receiver [1000;2000] to desired roll, pitch, yaw rate and throttle
void controlLoop_getDesiredInputs(){
  debug_t0[1] = micros();
  
  //RX_time_high[4] to roll_des, pitch_des, yaw_rate_des, throttle_des
  //map(value, fromLow, fromHigh, toLow, toHigh)

  pitch_des = map(RX_time_high[RX_1], RX_MAX, RX_MIN, ANGLE_MIN, ANGLE_MAX); // RX_MAX, RX_MIN
  roll_des = map(RX_time_high[RX_2], RX_MIN, RX_MAX, ANGLE_MIN, ANGLE_MAX);
  throttle_des = map(RX_time_high[RX_3], RX_MAX, RX_MIN, THRUST_MIN, THRUST_MAX); // RX_MAX, RX_MIN
  yaw_rate_des = map(RX_time_high[RX_4], RX_MIN, RX_MAX, RATE_MIN, RATE_MAX);
  
  //controlLoop_str_debug = "RX_time_high " + String(RX_time_high[RX_1]) + " " + String(RX_time_high[RX_2]) + " " + String(RX_time_high[RX_3]) + " " + String(RX_time_high[RX_4]);
  //debug_println_str(controlLoop_str_debug);
  //controlLoop_str_debug = "pitch_des " + String(pitch_des) + " roll_des " + String(roll_des) + " throttle_des " + String(throttle_des) + " yaw_rate_des " + String(yaw_rate_des);
  //debug_println_str(controlLoop_str_debug);
  debug_t1[1] = micros() - debug_t0[1];
}

// internal
// set the outputs from desired motor speed [rad/s] to escs [1000;2000]
void controlLoop_setESCoutputs(){
  debug_t0[2] = micros();
  
  //w1,w2,w3,w4 to esc1, esc2, esc3, esc4
  esc1=map(w1,W_MIN, W_MAX, ESC_MIN, ESC_MAX);
  esc2=map(w2,W_MIN, W_MAX, ESC_MIN, ESC_MAX);
  esc3=map(w3,W_MIN, W_MAX, ESC_MIN, ESC_MAX);
  esc4=map(w4,W_MIN, W_MAX, ESC_MIN, ESC_MAX);

  //controlLoop_str_debug = "w " + String(w1) + " " + String(w2) + " " + String(w3) + " " + String(w4);
  //debug_println_str(controlLoop_str_debug);
  //controlLoop_str_debug = "esc " + String(esc1) + " " + String(esc2) + " " + String(esc3) + " " + String(esc4);
  //debug_println_str(controlLoop_str_debug);
  debug_t1[2] = micros() - debug_t0[2];
}

// internal
// update rotor speeds based on desired rates and measured rates
void controlLoop_inner(){
  debug_t0[3] = micros();
  
  roll_rate_pid.Compute();
  pitch_rate_pid.Compute();
  yaw_rate_pid.Compute();

  w1 =  K*roll_rate_control + K*pitch_rate_control - K*yaw_rate_control + K_thrust*throttle_des;
  w2 = -K*roll_rate_control + K*pitch_rate_control + K*yaw_rate_control + K_thrust*throttle_des;
  w3 = -K*roll_rate_control - K*pitch_rate_control - K*yaw_rate_control + K_thrust*throttle_des;
  w4 =  K*roll_rate_control - K*pitch_rate_control + K*yaw_rate_control + K_thrust*throttle_des;

  debug_t1[3] = micros() - debug_t0[3];
}

// internal
// update desired rate based on desired angles and measured angles
void controlLoop_outer(){
  debug_t0[4] = micros();
  
  controlLoop_sensor_fusion();
  controlLoop_getDesiredInputs();
  roll_pid.Compute();
  pitch_pid.Compute();
  
  debug_t1[4] = micros() - debug_t0[4];
}

// internal
// compute measured angles from accelerometer, gyroscope and magnetometer fusion
void controlLoop_sensor_fusion(){
  debug_t0[5] = micros();
  
  IMU_getData();

	// TODO control x y z with roll pitch yaw axis

   // --------roll and pitch from acceleration----------------------- 
   //double atan2( double  __y,double  __x)  arc tangent of __y / __x,
   roll_meas=atan2(acc_y, acc_z);  // range [-pi, +pi] radians.
   pitch_meas=atan2(acc_x, sqrt(acc_y*acc_y+acc_z*acc_z)); // range [-pi, +pi] radians.

   controlLoop_str_debug = "roll_meas " + String(roll_meas) + " pitch_meas " + String(pitch_meas);
   debug_println_str(controlLoop_str_debug);
   
   // rate from gyroscope 
   roll_rate_meas = gyr_x;
   pitch_rate_meas = gyr_y;
   yaw_rate_meas = gyr_z;
   
   //controlLoop_str_debug = "roll_rate_meas " + String(roll_rate_meas) + " pitch_rate_meas " + String(pitch_rate_meas) + " yaw_rate_meas " + String(yaw_rate_meas);
   //debug_println_str(controlLoop_str_debug);
   debug_t1[5] = micros() - debug_t0[5];
}

void debug_printTime(){
  debug_println_str("timing of function");
  Serial.println(debug_t1[0]);
  controlLoop_str_debug = "controlLoop_loop " + String(debug_t1[0]);
  debug_println_str(controlLoop_str_debug);
  controlLoop_str_debug = "controlLoop_getDesiredInputs " + String(debug_t1[1]);
  debug_println_str(controlLoop_str_debug);
  controlLoop_str_debug = "controlLoop_setESCoutputs " + String(debug_t1[2]);
  debug_println_str(controlLoop_str_debug);
  controlLoop_str_debug = "controlLoop_inner " + String(debug_t1[3]);
  debug_println_str(controlLoop_str_debug);
  controlLoop_str_debug = "controlLoop_outer " + String(debug_t1[4]);
  debug_println_str(controlLoop_str_debug);
  controlLoop_str_debug = "controlLoop_sensor " + String(debug_t1[5]);
  debug_println_str(controlLoop_str_debug);
  controlLoop_str_debug = "IMU_getData " + String(debug_t1[6]);
  debug_println_str(controlLoop_str_debug);
}
