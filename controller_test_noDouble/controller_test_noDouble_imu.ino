#include "imu.h"
#include <Wire.h>
#include <TimerTwo.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

// internal
bool IMU_newDataAvailable = false;
uint8_t Buf[14];  // buffer for i2c read
int16_t ax, ay, az; // Accelerometer
int16_t gx, gy, gz; // Gyroscope

String imu_str_debug = "";

// external
void IMU_setup(){
  Wire.begin();
  debug_println_str("IMU_setup start");
  debug_println_str("IMU_setup acc");
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06); // reg 29 = 0x1D = ACCEL_CONFIG2
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06); // reg 26 = 0x1A = CONFIG

  debug_println_str("IMU_setup gyro");
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS); // reg 27 = 0x1B = GYRO_CONFIG
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G); // reg 28 = 0x1C = ACCEL_CONFIG
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  debug_println_str("IMU_setup mag");
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
  
  debug_println_str("IMU_setup timer2");
  // Initialize timer1, and set a 125 us period = 8kHz
  //Timer1.initialize(1250);
  // Attach IMU_dataAvailable_callback() as a timer overflow interrupt
  //Timer1.attachInterrupt(IMU_callback_newDataAvailable);
  // initialize timer2, and set a 125 us period = 8kHz and attach IMU_dataAvailable_callback()
  Timer2.EnableTimerInterrupt(IMU_callback_newDataAvailable, 125);

  IMU_calibration();
    
  debug_println_str("imu_setup done");
}

// internal
// calibration of the imu roll and pitch
void IMU_calibration(){

  // Read accelerometer and gyroscope
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Create 16 bits values from 8 bits data

  // Accelerometer
  ax=-(Buf[0]<<8 | Buf[1]);
  ay=-(Buf[2]<<8 | Buf[3]);
  az=Buf[4]<<8 | Buf[5];

  acc_x = ax;
  acc_y = ay;
  acc_z = az;
  
  // --------roll and pitch from acceleration----------------------- 
  //double atan2( double  __y,double  __x)  arc tangent of __y / __x,
  roll_calib=1000.*atan2(acc_y, acc_z);  // range 1000*[-pi, +pi] milli radians.
  pitch_calib=1000.*atan2(acc_x, sqrt(acc_y*acc_y+acc_z*acc_z)); // range 1000*[-pi, +pi] milli radians.

  debug_println_str("IMU_calibration done");
}

// internal
// Set boolean for saying that new data from IMU are available
void IMU_callback_newDataAvailable()
{ 
  IMU_newDataAvailable = true;
}

// external
// Get accel and gyro data from IMU
void IMU_getData(){
  //debug_t0[6] = micros();
  if(IMU_newDataAvailable){
    //debug_t0[7] = micros();
    // Read accelerometer and gyroscope
    I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
    
    // Create 16 bits values from 8 bits data
  
    // Accelerometer
    ax=-(Buf[0]<<8 | Buf[1]);
    ay=-(Buf[2]<<8 | Buf[3]);
    az=Buf[4]<<8 | Buf[5];

    acc_x = ax;
    acc_y = ay;
    acc_z = az;

    // Gyroscope
    gx=-(Buf[8]<<8 | Buf[9]);
    gy=-(Buf[10]<<8 | Buf[11]);
    gz=Buf[12]<<8 | Buf[13];

    // todo : fix scaling such that gyr_x = millirad/s
    gyr_x = gx>>2; // (double)(gx>>5) * (DEG_TO_MILLIRAD) = >>2
    gyr_y = gy>>2; //(double)(gy>>5) * (DEG_TO_MILLIRAD);
    gyr_z = gz>>2; //(double)(gz>>5) * (DEG_TO_MILLIRAD);

    // new data used, so no more new data
    IMU_newDataAvailable = false;
  }
}


// internal
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
  Data[index++]=Wire.read();
}

// internal
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

