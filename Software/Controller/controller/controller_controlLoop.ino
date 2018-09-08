#include <PID_long.h> // pid library
#include <math.h>   // math library
#include "esc.h"
#include "imu.h"
#include "receiver.h"

// internal
#define DEG_TO_MILLIRAD (17) // 1000*M_PI/180
#define MILLIRAD_TO_DEG (0.058) //(180./(1000*M_PI))

// internal
#define K_thrust 100
#define K 1

#define ANGLE_MIN (-45*DEG_TO_MILLIRAD) // corresponds to -45 deg en milli rad
#define ANGLE_MAX (45*DEG_TO_MILLIRAD)  // corresponds to  45 deg en milli rad
#define RATE_MIN (-4.5*DEG_TO_MILLIRAD)// corresponds to -4.5 deg/s en milli rad/s
#define RATE_MAX (4.5*DEG_TO_MILLIRAD)  // corresponds to  4.5 deg/s en milli rad/s
#define THRUST_MIN 0 // corresponds to 0 cm/s
#define THRUST_MAX 100  // corresponds to 100 cm/s
#define W_MIN 1000000 // millirad/s
#define W_MAX 10000000 // millirad/s

// internal
long roll_des, pitch_des; // milli rad
long roll_rate_des, pitch_rate_des, yaw_rate_des; // milli rad/s
long throttle_des; // sqrt(m/s)

long roll_meas, pitch_meas; // milli rad
long roll_rate_meas, pitch_rate_meas, yaw_rate_meas; // milli rad/s

// internal
long roll_rate_control, pitch_rate_control, yaw_rate_control; // milli rad/s
long w1, w2, w3, w4; // rad/s

// internal
long Kp=1, Ki=1, Kd=0;
PID roll_rate_pid(&roll_rate_meas, &roll_rate_control, &roll_rate_des, Kp, Ki, Kd);
PID pitch_rate_pid(&pitch_rate_meas, &pitch_rate_control, &pitch_rate_des, Kp, Ki, Kd);
PID yaw_rate_pid(&yaw_rate_meas, &yaw_rate_control, &yaw_rate_des, Kp, Ki, Kd);

PID roll_pid(&roll_meas, &roll_rate_des, &roll_des, Kp, Ki, Kd);
PID pitch_pid(&pitch_meas, &pitch_rate_des, &pitch_des, Kp, Ki, Kd);

int cnt = 10;

// external
void controlLoop_loop(){
  
  // outer loop 10 times less often than inner loop
  if(cnt == 10 ){
    controlLoop_outer();
    cnt = 0;
  }
  cnt ++;

  // inner loop
  controlLoop_inner();
  controlLoop_setESCoutputs();
}

// internal
// update desired rate based on desired angles and measured angles
void controlLoop_outer(){
  controlLoop_sensor_fusion();
  controlLoop_getDesiredInputs();
  
  roll_pid.Compute();
  pitch_pid.Compute();
}

// internal
// compute measured angles from accelerometer, gyroscope and magnetometer fusion
void controlLoop_sensor_fusion(){
  
  IMU_getData();

  // TODO control x y z with roll pitch yaw axis

  // --------roll and pitch from acceleration----------------------- 
  //double atan2( double  __y,double  __x)  arc tangent of __y / __x,
  roll_meas=1000.*atan2(acc_y, acc_z);  // range 1000*[-pi, +pi] milli radians.
  pitch_meas=1000.*atan2(acc_x, sqrt(acc_y*acc_y+acc_z*acc_z)); // range 1000*[-pi, +pi] milli radians.

  // use calibrate values
  roll_meas = roll_meas - roll_calib;
  pitch_meas = pitch_meas - pitch_calib;
  
  // rate from gyroscope 
  roll_rate_meas = gyr_x;
  pitch_rate_meas = gyr_y;
  yaw_rate_meas = gyr_z;  
}

// internal
// get the inputs from receiver [1000;2000] to desired roll, pitch, yaw rate and throttle
void controlLoop_getDesiredInputs(){  
  //RX_time_high[4] to roll_des, pitch_des, yaw_rate_des, throttle_des
  //mapped = map(value, fromLow, fromHigh, toLow, toHigh)

  pitch_des = map(RX_time_high[RX_1], RX_MAX, RX_MIN, ANGLE_MIN, ANGLE_MAX); // RX_MAX, RX_MIN
  roll_des = map(RX_time_high[RX_2], RX_MIN, RX_MAX, ANGLE_MIN, ANGLE_MAX);
  throttle_des = map(RX_time_high[RX_3], RX_MAX, RX_MIN, THRUST_MIN, THRUST_MAX); // RX_MAX, RX_MIN
  yaw_rate_des = map(RX_time_high[RX_4], RX_MIN, RX_MAX, RATE_MIN, RATE_MAX);
}

// internal
// update rotor speeds based on desired rates and measured rates
void controlLoop_inner(){
  
  roll_rate_pid.Compute();
  pitch_rate_pid.Compute();
  yaw_rate_pid.Compute();

  w1 =  K*roll_rate_control + K*pitch_rate_control - K*yaw_rate_control + K_thrust*throttle_des;
  w2 = -K*roll_rate_control + K*pitch_rate_control + K*yaw_rate_control + K_thrust*throttle_des;
  w3 = -K*roll_rate_control - K*pitch_rate_control - K*yaw_rate_control + K_thrust*throttle_des;
  w4 =  K*roll_rate_control - K*pitch_rate_control + K*yaw_rate_control + K_thrust*throttle_des;
}

// internal
// set the outputs from desired motor speed [millirad/s] to escs ~[1000;2000]
void controlLoop_setESCoutputs(){
  
  //w1,w2,w3,w4 to esc1, esc2, esc3, esc4
  esc1=map(w1,W_MIN, W_MAX, ESC_MIN, ESC_MAX);
  esc2=map(w2,W_MIN, W_MAX, ESC_MIN, ESC_MAX);
  esc3=map(w3,W_MIN, W_MAX, ESC_MIN, ESC_MAX);
  esc4=map(w4,W_MIN, W_MAX, ESC_MIN, ESC_MAX);

  esc_write_all();
}

void control_loop_initPIDs(){
  roll_rate_pid.Start();
  pitch_rate_pid.Start();
  yaw_rate_pid.Start();
  roll_pid.Start();
  pitch_pid.Start();
}






