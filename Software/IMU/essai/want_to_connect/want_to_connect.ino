#include <Wire.h>

#define MPU_ADDR 0x68
#define MAGN_ADDR 0x0C
#define WHO_I_AM  0x75
#define MAG_ID  0x00
#define MAG_STATUS  0x02
#define MAG_DATA  0x03

#define SAMPLE_RATE_DIVIDER 25
#define GYRO_CONFIG 27
#define ACCEL_CONFIG  28
#define ACCEL_CONFIG2 29
#define INT_PIN_CFG 0x37
#define INT_ENABLE  0x38
#define INT_STATUS  0x39

#define SMPLRT_DIV 0x0
#define BYPASS  0x02

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define DATA_ADDR 0x3B

#define DATA_NB_BYTE  14
#define MAG_NB_BYTE   7

int I2Cread(uint8_t address, uint8_t Register, uint8_t nbyte, uint8_t* data) {
  int error;
  Wire.beginTransmission(address);
  Wire.write(Register);
  error=Wire.endTransmission(false);
  if(error==0) {
    Wire.requestFrom(address,nbyte);
    uint8_t index=0;
    while(Wire.available()) {
      data[index++]=Wire.read();
    }
  }
  return error;
}

int I2Cwrite(uint8_t address, uint8_t Register, uint8_t data) {
  int error;
  Wire.beginTransmission(address);
  Wire.write(Register);
  Wire.write(data);
  error=Wire.endTransmission();
  return error;
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(38400);
  while(!Serial);
  uint8_t data;
  I2Cread(MPU_ADDR,WHO_I_AM,1,&data);
  Serial.println(data,HEX);
  if(data!=0x71){
    Serial.println(F("Communication failed with MPU, abort!"));
    Serial.flush();
    abort();
  }
  data=0;
  I2Cread(MAGN_ADDR,MAG_ID,1,&data);
  Serial.println(data,HEX);
  if(data!=0x48) {
    Serial.println(F("Communication failed with Magn, abort!"));
    Serial.flush();
    abort();
  }
  I2Cwrite(MPU_ADDR,GYRO_CONFIG,GYRO_FULL_SCALE_2000_DPS);
  I2Cwrite(MPU_ADDR,ACCEL_CONFIG,ACC_FULL_SCALE_16_G);
  I2Cwrite(MPU_ADDR,INT_PIN_CFG,BYPASS);
}

long int cpt=0;
void loop() {
  // put your main code here, to run repeatedly:
  uint8_t Buff[14];
  I2Cread(MPU_ADDR,DATA_ADDR, DATA_NB_BYTE, Buff);

  // Accelerometer
  int16_t ax=-(Buff[0]<<8 | Buff[1]);
  int16_t ay=-(Buff[2]<<8 | Buff[3]);
  int16_t az=Buff[4]<<8 | Buff[5];

  // Gyroscope
  int16_t gx=-(Buff[8]<<8 | Buff[9]);
  int16_t gy=-(Buff[10]<<8 | Buff[11]);
  int16_t gz=Buff[12]<<8 | Buff[13];

  Serial.print("ax= ");
  Serial.println(ax,DEC);
  Serial.print("ay= ");
  Serial.println(ay,DEC);
  Serial.print("az= ");
  Serial.println(az,DEC);
  Serial.print("\n");

  Serial.print("gx= ");
  Serial.println(gx,DEC);
  Serial.print("gy= ");
  Serial.println(gy,DEC);
  Serial.print("gz= ");
  Serial.println(gz,DEC);
  Serial.print("\n");

  uint8_t stat;
  uint8_t bufMag[MAG_NB_BYTE];
  do {
    I2Cread(MAGN_ADDR,MAG_STATUS,1,&stat);
  }while(!(stat&0x01));

  I2Cread(MAGN_ADDR,MAG_DATA,MAG_NB_BYTE,bufMag);

   // Magnetometer
  int16_t mx=-(bufMag[3]<<8 | bufMag[2]);
  int16_t my=-(bufMag[1]<<8 | bufMag[0]);
  int16_t mz=-(bufMag[5]<<8 | bufMag[4]);
  
  
  // Magnetometer
  Serial.print ("mx= ");
  Serial.println (mx+200,DEC); 
  Serial.print ("my= ");
  Serial.println (my-70,DEC);
  Serial.print ("mz= ");
  Serial.println (mz-700,DEC);  
  Serial.print ("\n");
}
