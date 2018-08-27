#include <PID_vDITRIA.h> // pid library
#include <math.h>   // math library

#define K 10
#define ANGLE_MIN -450 // corresponds to -45 deg
#define ANGLE_MAX 450  // corresponds to  45 deg
#define RATE_MIN -450 // corresponds to -4.5 deg/s
#define RATE_MAX 450  // corresponds to  4.5 deg/s
#define THRUST_MIN -450 // corresponds to -45 cm/s
#define THRUST_MAX 450  // corresponds to 45 cm/s

#define DEG_TO_RAD (M_PI/180.)
#define RAD_TO_DEG (180./M_PI)

int radio_data_1, radio_data_2, radio_data_3, radio_data_4; // range [1000,2000] received from radio

double roll_des, pitch_des; // rad
double roll_rate_des, pitch_rate_des, yaw_rate_des; // rad/s
double throttle_des; // sqrt(m/s)

double roll_meas, pitch_meas; // rad
double roll_rate_meas, pitch_rate_meas, yaw_rate_meas; // rad/s
double acc_x, acc_y, acc_z; // m/s

double roll_rate_control, pitch_rate_control, yaw_rate_control; // rad/s

double w1, w2, w3, w4; // rad/s
int wmot_1, wmot_2, wmot_3, wmot_4; // range [1000,2000] for sending to esc

double sim_roll, sim_pitch, sim_yaw;

double Kp=2, Ki=5, Kd=1;
PID roll_rate_pid(&roll_rate_meas, &roll_rate_control, &roll_rate_des, Kp, Ki, Kd);
PID pitch_rate_pid(&pitch_rate_meas, &pitch_rate_control, &pitch_rate_des, Kp, Ki, Kd);
PID yaw_rate_pid(&yaw_rate_meas, &yaw_rate_control, &yaw_rate_des, Kp, Ki, Kd);

PID roll_pid(&roll_meas, &roll_rate_des, &roll_des, Kp, Ki, Kd);
PID pitch_pid(&pitch_meas, &roll_rate_des, &pitch_des, Kp, Ki, Kd);

String toPrint = "";
int i=0;
long int t_prev;
double dt;

void setup() {
  Serial.begin(9600);
  
  acc_x=0, acc_y=0, acc_z=9.81;
  roll_des=45*DEG_TO_RAD;
  pitch_des=20*DEG_TO_RAD;
  
  t_prev=millis();
}

void loop() {
  i++;
  if(i==10){
    outer_controller();
    i=0;
  }
  inner_controller();
  dt=(millis()-t_prev)/1000;
  model_sim();
  write_data();
}

// update rotor speeds based on desired rates and measured rates
void inner_controller(){
  roll_rate_pid.Compute();
  pitch_rate_pid.Compute();
  yaw_rate_pid.Compute();

  w1= K*roll_rate_control + K*pitch_rate_control - K*yaw_rate_control + K*throttle_des;
  w2=-K*roll_rate_control + K*pitch_rate_control + K*yaw_rate_control + K*throttle_des;
  w3=-K*roll_rate_control - K*pitch_rate_control - K*yaw_rate_control + K*throttle_des;
  w4= K*roll_rate_control - K*pitch_rate_control + K*yaw_rate_control + K*throttle_des;
}

// update desired rate based on desired angles and measured angles
void outer_controller(){
  sensor_fusion();
  roll_pid.Compute();
  pitch_pid.Compute();
}

// compute measured angles from accelerometer, gyroscope and magnetometer fusion
void sensor_fusion(){

   // --------roll and pitch from acceleration----------------------- 
   //double atan2( double  __y,double  __x)  arc tangent of __y / __x,
   roll_meas=atan2(acc_y, acc_z);  // range [-pi, +pi] radians.
   pitch_meas=atan2(acc_x, sqrt(acc_y*acc_y+acc_z*acc_z)); // range [-pi, +pi] radians.
}

void write_data(){
  Serial.println("loop :----------");
  Serial.println("acc_x acc_y acc_z");
  toPrint = "";
  toPrint += acc_x;
  toPrint += " ";
  toPrint += acc_y;
  toPrint += " ";
  toPrint += acc_z;
  toPrint += "\n";
  Serial.println(toPrint);
  
  Serial.println("roll_des roll_meas roll_rate_des");
  toPrint = "";
  toPrint += roll_des;
  toPrint += " ";
  toPrint += roll_meas;
  toPrint += " ";
  toPrint += roll_rate_des;
  toPrint += "\n";
  Serial.println(toPrint);
  
  Serial.println("pitch_des pitch_meas pitch_rate_des");
  toPrint = "";
  toPrint += pitch_des;
  toPrint += " ";
  toPrint += pitch_meas;
  toPrint += " ";
  toPrint += pitch_rate_des;
  toPrint += "\n";
  Serial.println(toPrint);

  Serial.println("w1 w2 w3 w4");
  toPrint = "";
  toPrint += w1;
  toPrint += " ";
  toPrint += w2;
  toPrint += " ";
  toPrint += w3;
  toPrint += " ";
  toPrint += w4;
  toPrint += "\n";
  Serial.println(toPrint);
}


void model_sim(){
  sim_roll+=K*(w1+w4-w2-w3)*dt; 
  sim_pitch+=K*(w1+w2-w3-w4)*dt;
  sim_yaw=K*(w2+w4-w1-w3)*dt;

  acc_x=-(cos(sim_yaw)*sin(sim_pitch)*cos(sim_roll)+sin(sim_yaw)*sin(sim_roll))*9.81;
  acc_y=-(sin(sim_yaw)*sin(sim_pitch)*cos(sim_roll)-cos(sim_yaw)*sin(sim_roll))*9.81;
  acc_z=-(cos(sim_pitch)*cos(sim_roll))*9.81;
}

