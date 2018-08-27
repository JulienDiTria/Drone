#include <Wire.h>
#include <TimerOne.h>

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

#define DEG_TO_RAD (M_PI/180.)

bool bool_getDataIMU = false;
int cnt = 0; // compteur de frequence
uint8_t Buf[14];  // buffer for i2c read
int16_t ax, ay, az; // Accelerometer
int16_t gx, gy, gz; // Gyroscope
int16_t gx_avg, gy_avg, gz_avg;


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
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


void setup() {
  // Arduino initializations
  Wire.begin();
  Serial.begin(9600);
  //Serial.println("begin setup");

  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06); // reg 29 = 0x1D = ACCEL_CONFIG2
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06); // reg 26 = 0x1A = CONFIG

  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS); // reg 27 = 0x1B = GYRO_CONFIG
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G); // reg 28 = 0x1C = ACCEL_CONFIG
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);

  Timer1.initialize(125);         // initialize timer1, and set a 125 us period = 8kHz
  //Serial.println("timer1 init");
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  //Serial.println("end setup");
}
unsigned long t0, t1;
int count = 0 ;

void loop() {

  if (bool_getDataIMU) {
    bool_getDataIMU = false;
    // Read accelerometer and gyroscope
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
    // Create 16 bits values from 8 bits data

    // Accelerometer
    ax = -(Buf[0] << 8 | Buf[1]);
    ay = -(Buf[2] << 8 | Buf[3]);
    az = Buf[4] << 8 | Buf[5];

    // Gyroscope
    gx = -(Buf[8] << 8 | Buf[9]);
    gy = -(Buf[10] << 8 | Buf[11]);
    gz = Buf[12] << 8 | Buf[13];

    // Accelerometer
    //Serial.print ("acc ");
    //    Serial.print(ax,DEC);
    //    Serial.print ("\t");
    //    Serial.print(ay,DEC);
    //    Serial.print ("\t");
    //    Serial.print(az,DEC);
    //    Serial.print ("\t");

    // Gyroscope
    //Serial.print ("gyr ");
    //    Serial.print (gx,DEC);
    //    Serial.print ("\t");
    //    Serial.print (gy,DEC);
    //    Serial.print ("\t");
    Serial.print (double(gz >> 11));
    Serial.print ("\t");
    Serial.print (double((gz / 32) >> 6));
    Serial.print ("\t");
    Serial.print (double((gz >> 5)*DEG_TO_RAD));
    Serial.print ("\t");
    Serial.print (double(DEG_TO_RAD * gz / 32));
    Serial.print ("\n");

  }

}

void callback()
{
  bool_getDataIMU = true;
}

