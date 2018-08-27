// pid library
#include <PID_v1.h>

#define K 10

enum{ROLL, PITCH, YAW, THRUST};

// input
double state_desired[4]; // desired state coming from radio : Roll angle, Pitch angle, Yaw Rate and Thrust
double state_measured[2]; // measured Roll angle and Pitch angle
double rate_measured[3]; // measure Roll rate, Pitch rate, Yaw rate


// output
double w[4]; // motor speed

//Specify the links and initial tuning parameters
double Kp=0, Ki=5, Kd=0;

void setup() {
  // put your setup code here, to run once:
  state_desired[ROLL]=1024;
  state_desired[PITCH]=1024;
  state_desired[YAW]=1024;
  state_desired[THRUST]=1024;

  state_measured[ROLL]=2000;
  state_measured[PITCH]=1500;

  rate_measured[ROLL]=1;
  rate_measured[PITCH]=5;
  rate_measured[YAW]=0;
}

void loop() {
  // put your main code here, to run repeatedly:
  
}

// input  : - desired thrust and desired roll angle, pitch angle, yaw angle
//          - measured roll angle and pitch angle
//          - measured roll rate, pitch rate and yaw rate
// output : 4 rotors speed vector
void angle_controllers(double w[4], double state_desired[4], double state_measured[2], double rate_measured[3]){
  
}

// input  : - desired thrust and desired roll_rate, pitch_rate, yaw_rate
//          - measured roll_rate, pitch_rate, yaw_rate
// output : 4 rotors speed vector
void rate_controllers(double w[4], double rate_des[4], double rate_meas[3]){
  double rate_des_control[4];
  
  PID roll_rate_controller(&(rate_meas[ROLL]), &(rate_des_control[ROLL]), &(rate_des[ROLL]), Kp, Ki, Kd, DIRECT);
  PID pitch_rate_controller(&(rate_meas[PITCH]), &(rate_des_control[PITCH]), &(rate_des[PITCH]), Kp, Ki, Kd, DIRECT);
  PID yaw_rate_controller(&(rate_meas[YAW]), &(rate_des_control[YAW]), &(rate_des[YAW]), Kp, Ki, Kd, DIRECT);
  
  roll_rate_controller.SetMode(AUTOMATIC);
  pitch_rate_controller.SetMode(AUTOMATIC);
  yaw_rate_controller.SetMode(AUTOMATIC);

  roll_rate_controller.Compute();
  pitch_rate_controller.Compute();
  yaw_rate_controller.Compute();
  rate_des_control[THRUST]=rate_des[THRUST];

  angularRateToRotorSpeed(w, rate_des_control);
}

// input  : desired thrust and controlled output roll_rate, pitch_rate, yaw_rate
// output : 4 rotors speed vector
void angularRateToRotorSpeed(double w[4], double rate_des_control[4]){
  w[0]= K*rate_des_control[ROLL] + K*rate_des_control[PITCH] - K*rate_des_control[YAW] + K*rate_des_control[THRUST];
  w[1]=-K*rate_des_control[ROLL] + K*rate_des_control[PITCH] + K*rate_des_control[YAW] + K*rate_des_control[THRUST];
  w[2]=-K*rate_des_control[ROLL] - K*rate_des_control[PITCH] - K*rate_des_control[YAW] + K*rate_des_control[THRUST];
  w[3]= K*rate_des_control[ROLL] - K*rate_des_control[PITCH] + K*rate_des_control[YAW] + K*rate_des_control[THRUST];
}


