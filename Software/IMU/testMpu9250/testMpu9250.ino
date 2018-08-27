#include "quaternionFilters.h"
#include "MPU9250.h"

MPU9250 myIMU;

// Pin definitions
int intPin = 2;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

void setup() {

  Serial.begin(9600);
  // Wait for input
  Serial.println("Waiting any input for start");
  while (!Serial.available());
  Serial.read();
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  // Start by performing self test and reporting values
  myIMU.MPU9250SelfTest(myIMU.selfTest);
  Serial.print("x-axis self test: acceleration trim within : ");
  Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: acceleration trim within : ");
  Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: acceleration trim within : ");
  Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
  Serial.print("x-axis self test: gyration trim within : ");
  Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: gyration trim within : ");
  Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: gyration trim within : ");
  Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

  // Calibrate gyro and accelerometers, load biases in bias registers
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

   myIMU.initMPU9250();
  // Initialize device for active mode read of acclerometer, gyroscope, and
  // temperature
  Serial.println("MPU9250 initialized for active data mode....");

}

void loop() {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    // Print acceleration values in milligs!
    //Serial.print("X-acceleration: "); 
    Serial.print(1000 * myIMU.ax);
    Serial.print(" ");
    //Serial.print(" mg ");
    //Serial.print("Y-acceleration: "); 
    Serial.print(1000 * myIMU.ay);
    Serial.print(" ");
    //Serial.print(" mg ");
    //Serial.print("Z-acceleration: "); 
    Serial.print(1000 * myIMU.az);
    Serial.print(" ");
    Serial.println(" mg ");

    // Print gyro values in degree/sec
    //Serial.print("X-gyro rate: "); 
    Serial.print(myIMU.gx, 3);
    Serial.print(" ");
    //Serial.print(" degrees/sec ");
    //Serial.print("Y-gyro rate: "); 
    Serial.print(myIMU.gy, 3);
    Serial.print(" ");
    //Serial.print(" degrees/sec ");
    //Serial.print("Z-gyro rate: "); 
    Serial.print(myIMU.gz, 3);
    Serial.println(" degrees/sec");
  }

}
