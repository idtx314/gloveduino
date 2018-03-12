 // MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
#include "MPU9250.h"
#include <Sparkfun_DRV2605L.h> //SparkFun Haptic Motor Driver Library 


const int MPU_addr=0x68;  // I2C address of the MPU-6050
const int AK_addr=0x0C; // I2C address of the AK8963 Magnetometer registers are denoted by an H at the end.
uint8_t c;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

MPU9250 myIMU;        //Initialize class
SFE_HMD_DRV2605L HMD; //Create haptic motor driver object 

void setup(){
  Wire.begin();
  //Initialize Serial
  Serial.begin(9600);

  //Power on MPU
//  Wire.beginTransmission(MPU_addr);
//  Wire.write(0x6B);  // PWR_MGMT_1 register
//  Wire.write(0);     // set to zero (wakes up the MPU-6050)
//  Wire.endTransmission(true);
  myIMU.initMPU9250();    //debug Including this line allows the magnetometer to collect data too. Not sure why.

  //Power on Magneto
  //  Wire.beginTransmission(AK_addr);
  //  Wire.write(0x0A); //CNTL1 register
  //  Wire.write(0x12); //16 bit, 8 HZ mode, use high nibble = 0000 for 14 bit, low nibble = 0110 for 100Hz
  //  Wire.endTransmission(true);
  myIMU.initAK8963(myIMU.magCalibration);   //Does above, plus testing and calibrating. myIMU.magCalibration contains cal data
  magcalMPU9250(myIMU.magbias, myIMU.magScale);
  myIMU.getMres();  //sets the mRes global variable to convert 14 or 16 bit resolution based on board config
  Serial.println("AK8963 mag biases (mG)"); Serial.println(myIMU.magbias[0]); Serial.println(myIMU.magbias[1]); Serial.println(myIMU.magbias[2]); 
  Serial.println("AK8963 mag scale (mG)"); Serial.println(myIMU.magScale[0]); Serial.println(myIMU.magScale[1]); Serial.println(myIMU.magScale[2]); 
  Serial.print("X-Axis sensitivity adjustment value "); Serial.println(myIMU.magCalibration[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(myIMU.magCalibration[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(myIMU.magCalibration[2], 2);
  delay(2000); // add delay to see results before serial spew of data
  
  //Initialize HMD
  HMD.begin();
  HMD.Mode(0); // Internal trigger input mode -- Must use the GO() function to trigger playback.
  HMD.MotorSelect(0x36); // ERM motor, 4x Braking, Medium loop gain, 1.365x back EMF gain
  HMD.Library(2); //1-5 & 7 for ERM library A-E & F, 6 for LRA library. Varies rate/overdrive voltage, rise time, brake time

}

void loop(){
  //Request section
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers

  //Receive Section
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //Mag Request
  Wire.beginTransmission(AK_addr);
  Wire.write(0x03);  // starting with register 0x03H (MAG_XOUT_L)
  Wire.endTransmission(false);
  Wire.requestFrom(AK_addr,7,true);  // request a total of 6 registers

  float magnitude;
  
  //Mag Receive
  myIMU.magCount[0]=Wire.read()|Wire.read()<<8;  // 0x03H (MAG_XOUT_L) & 0x04H (MAG_XOUT_H)     
  myIMU.magCount[1]=Wire.read()|Wire.read()<<8;  // 0x05H (MAG_YOUT_L) & 0x06H (MAG_YOUT_H)  
  myIMU.magCount[2]=Wire.read()|Wire.read()<<8;  // 0x07H (MAG_ZOUT_L) & 0x08H (MAG_ZOUT_H)  
  c=Wire.read();  //Sensor overflow register, must be read to signal read end. Pg51.

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental
  // corrections
  // Get actual magnetometer value, this depends on scale being set
  // Reading* Resolution conversion factor * Calibration factor - bias determined by function

  //Hard Iron Correction and Calibration
  myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] - myIMU.magbias[0];
  myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] - myIMU.magbias[1];
  myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] - myIMU.magbias[2];

  //Soft Iron Correction
  myIMU.mx *= myIMU.magScale[0];
  myIMU.my *= myIMU.magScale[1];
  myIMU.mz *= myIMU.magScale[2];


  //todo make sure this can actually hold the full calculation. Might be too small?
  magnitude = sqrt(myIMU.mx*myIMU.mx + myIMU.my*myIMU.my + myIMU.mz*myIMU.mz);  //Calculate the field magnitude with math.h

  //Output
//  Serial.print("AcX = "); Serial.print(AcX);
//  Serial.print("\t| AcY = "); Serial.print(AcY);
//  Serial.print("\t| AcZ = "); Serial.print(AcZ);
//  Serial.print("\t| Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//  Serial.print("\t| GyX = "); Serial.print(GyX);
//  Serial.print("\t| GyY = "); Serial.print(GyY);
//  Serial.print("\t| GyZ = "); Serial.print(GyZ);
  Serial.print("\t| mx = "); Serial.print(myIMU.mx);
  Serial.print("\t| my = "); Serial.print(myIMU.my);
  Serial.print("\t| mz = "); Serial.print(myIMU.mz);
  Serial.print("\t| Magnitude = "); Serial.println(magnitude);
  delay(333);
  /* //Prototype Send  
  char msg[50] = "";
  sprintf(msg,"%f,%f,%f,%f\n",myIMU.mx, myIMU.my, myIMU.mz, magnitude);
  Serial.print(msg);
   */
  

  //Data Processing
  //Motor Command Section
  if(0)//(magnitude > 0) //todo re-enable
  {
    HMD.Waveform(0, 13); //loads wave into sequence register 0+seq
    HMD.go(); //Plays the sequence registers.
    delay(600);   //Without a delay, the motor quickly stops vibrating. Possibly a safety check or race condition?
  }
}

void magcalMPU9250(float * dest1, float * dest2) //hand in bias and Scale
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
    // shoot for ~fifteen seconds of mag data
    if(myIMU.readModeData() == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(myIMU.readModeData() == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
   for(ii = 0; ii < sample_count; ii++) {
    myIMU.readMagData(mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if(myIMU.readModeData() == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(myIMU.readModeData() == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*myIMU.mRes*myIMU.magCalibration[0];  // save mag biases in G (not mG?) for main program
    dest1[1] = (float) mag_bias[1]*myIMU.mRes*myIMU.magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*myIMU.mRes*myIMU.magCalibration[2];  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
  
   Serial.println("Mag Calibration done!");
}
