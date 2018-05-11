/* Magnetic Sensing Glove Microprocessor Code
 * 
 * This code is tested on a Sparkfun Redboard, communicating with an MPU9250 9-axis IMU via a 5V/3.3V logic level
 * converter and a DRV2605L Haptic Motor Driver breakout controlling a simple ERM motor. To use the visualizing
 * functions the Redboard should be connected via USB to a linux computer running ROS nodes available at 
 * https://github.com/idtx314/rosglove
 * 
 * Thanks to JohnChi https://playground.arduino.cc/Main/MPU-6050
 * and Kris Winer https://github.com/kriswiner/MPU9250
 * for examples of how to use the functions and operate the IMU over I2C.
 */

 
#include<Wire.h>
#include "MPU9250.h"            //SparkFun MPU-9250 Library
#include <Sparkfun_DRV2605L.h>  //SparkFun Haptic Motor Driver Library 

int debug = 0;
int ledO = 0;
const int MPU_addr=0x68;          // I2C address of the MPU-6050
const int AK_addr=0x0C;           // I2C address of the AK8963 Magnetometer registers are denoted by an H at the end.
uint8_t c;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float magnitude;
float prev_roll=0, out_roll=0;
float prev_pitch=0, out_pitch=0;
float prev_yaw=0, out_yaw=0;

MPU9250 myIMU;                    //Initialize class
SFE_HMD_DRV2605L HMD;             //Create haptic motor driver object 


void setup(){

//  pinMode(LED_BUILTIN, OUTPUT);   //TODO turn back on
  //Initialize I2C
  Wire.begin();
  //Initialize Serial
  Serial.begin(9600);

  // Perform a self-test routine to validate the Gyro and Accelerometer are working within 14% of factory "trim".
  myIMU.MPU9250SelfTest(myIMU.SelfTest);
  if(debug)
  {
  Serial.print("x-axis self test: acceleration trim within : "); Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: acceleration trim within : "); Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: acceleration trim within : "); Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
  Serial.print("x-axis self test: gyration trim within : "); Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: gyration trim within : "); Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: gyration trim within : "); Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");
  }

  // Calibrate Gyro and Accelerometers. Store biases in input variables.
  myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
  if(debug)
  {
  Serial.print("x-axis biases: "); Serial.print(myIMU.gyroBias[0]); Serial.print("d/s, "); Serial.print(myIMU.accelBias[0]); Serial.println("g");
  Serial.print("y-axis biases: "); Serial.print(myIMU.gyroBias[1]); Serial.print("d/s, "); Serial.print(myIMU.accelBias[1]); Serial.println("g");
  Serial.print("z-axis biases: "); Serial.print(myIMU.gyroBias[2]); Serial.print("d/s, "); Serial.print(myIMU.accelBias[2]); Serial.println("g");
  }


  //Power on IMU
  //  Wire.beginTransmission(MPU_addr);
  //  Wire.write(0x6B);  // PWR_MGMT_1 register
  //  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  //  Wire.endTransmission(true);
  myIMU.initMPU9250();

  //Power on Magnetometer
  //  Wire.beginTransmission(AK_addr);
  //  Wire.write(0x0A); //CNTL1 register
  //  Wire.write(0x12); //16 bit, 8 HZ mode, use high nibble = 0000 for 14 bit, low nibble = 0110 for 100Hz
  //  Wire.endTransmission(true);
  myIMU.initAK8963(myIMU.magCalibration);   //Does above, plus testing and calibrating. Store data in myIMU.magCalibration.

  //Call Magnetometer Calibration function, store calibration values in input variables.
  magcalMPU9250(myIMU.magbias, myIMU.magScale);
  if(1) //Todo relegate this to debug mode
  {
  Serial.println("AK8963 mag biases (mG)"); Serial.println(myIMU.magbias[0]); Serial.println(myIMU.magbias[1]); Serial.println(myIMU.magbias[2]); 
  Serial.println("AK8963 mag scale (mG)"); Serial.println(myIMU.magScale[0]); Serial.println(myIMU.magScale[1]); Serial.println(myIMU.magScale[2]); 
  Serial.print("X-Axis sensitivity adjustment value "); Serial.println(myIMU.magCalibration[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(myIMU.magCalibration[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(myIMU.magCalibration[2], 2);
  }


  //Get Conversion factors. These are used to convert data based on the resolution settings of the IMU. Values stored in myIMU.aRes, .gRes, and .mRes
  myIMU.getAres();
  myIMU.getGres();
  myIMU.getMres();


  //Initialize HMD
  HMD.begin();
  HMD.Mode(0);                  // Internal trigger input mode -- Must use the GO() function to trigger playback.
  HMD.MotorSelect(0x36);        // ERM motor, 4x Braking, Medium loop gain, 1.365x back EMF gain
  HMD.Library(2);               //1-5 & 7 for ERM library A-E & F, 6 for LRA library. Varies rate/overdrive voltage, rise time, brake time

}

void loop(){
//  //Heartbeat
//  if(ledO == 0)
//    ledO = 1;
//  else
//    ledO=0;
//
//  digitalWrite(LED_BUILTIN, ledO);    TODO turn heartbeat back on
  
  //Request section
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);                                                                         // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);                                                       // request a total of 14 registers

  //Receive Accel and Gyro ADC values
  AcX=Wire.read()<<8|Wire.read();                                                           // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();                                                           // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();                                                           // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();                                                           // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();                                                           // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();                                                           // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();                                                           // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //Data translation into gravities and degrees/second
  myIMU.ax = (float)AcX * myIMU.aRes - myIMU.accelBias[0];                                  //Ares and accelBias are floats, AcX is an int.
  myIMU.ay = (float)AcY * myIMU.aRes - myIMU.accelBias[1];
  myIMU.az = (float)AcZ * myIMU.aRes - myIMU.accelBias[2];
  myIMU.gx = (float)GyX * myIMU.gRes;
  myIMU.gy = (float)GyY * myIMU.gRes;
  myIMU.gz = (float)GyZ * myIMU.gRes;

  //Mag Request
  Wire.beginTransmission(AK_addr);
  Wire.write(0x03);                                                                         //Starting with register 0x03H (MAG_XOUT_L)
  Wire.endTransmission(false);
  Wire.requestFrom(AK_addr,7,true);                                                         //Request a total of 6 registers
  
  //Mag Receive
  myIMU.magCount[0]=Wire.read()|Wire.read()<<8;                                             // 0x03H (MAG_XOUT_L) & 0x04H (MAG_XOUT_H)     
  myIMU.magCount[1]=Wire.read()|Wire.read()<<8;                                             // 0x05H (MAG_YOUT_L) & 0x06H (MAG_YOUT_H)  
  myIMU.magCount[2]=Wire.read()|Wire.read()<<8;                                             // 0x07H (MAG_ZOUT_L) & 0x08H (MAG_ZOUT_H)  
  c=Wire.read();                                                                            // Sensor overflow register, must be read to signal end of read. Pg51.

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental
  // corrections
  // Get actual magnetometer value, this depends on scale being set
  // Reading* Resolution conversion factor * Calibration factor - bias determined by function

  //Apply Hard Iron Correction and Calibration
  myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] - myIMU.magbias[0];
  myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] - myIMU.magbias[1];
  myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] - myIMU.magbias[2];

  //Apply Soft Iron Correction
  myIMU.mx *= myIMU.magScale[0];
  myIMU.my *= myIMU.magScale[1];
  myIMU.mz *= myIMU.magScale[2];


  //Data Processing
  unsigned long current_time, vibe_timer=0;
  float time_diff, accel_roll_angle, accel_pitch_angle, A = .1;
  static float gyro_roll_angle=0, gyro_pitch_angle=0, gyro_yaw_angle=0;
  static unsigned long prev_time = 0;
  
  
  //Todo make sure this can actually hold the full calculation. Might be too small?
  magnitude = sqrt(myIMU.mx*myIMU.mx + myIMU.my*myIMU.my + myIMU.mz*myIMU.mz);                //Calculate the field magnitude with math.h


  //Rolling Sum of gyro orientation
    //Calculate Dt
  current_time = millis();                                                                    //Returns milliseconds. Runs for 50 days before overflowing
  time_diff = current_time - prev_time;
  time_diff = time_diff/1000.0;                                                               //Convert to seconds. Seems to be about .02 usually.
  prev_time = current_time;

    //Calculate Rolling Sums in degrees
  gyro_roll_angle = gyro_roll_angle + myIMU.gx*time_diff;
  gyro_pitch_angle = gyro_pitch_angle + myIMU.gy*time_diff;
  gyro_yaw_angle = gyro_yaw_angle + myIMU.gz*time_diff;
    
    
    //Accelerometer Orientation Estimate in degrees
    //atan2(double y, double x); This returns in rad/s, must be converted to degrees.
  accel_roll_angle = atan2((double)myIMU.ay,(double)myIMU.az) * 180/3.1415;
  accel_pitch_angle = atan2((double)myIMU.ax,(double)myIMU.az) * 180/3.1415;

    
    //Complementary Filter in degrees
  myIMU.roll  = A*accel_roll_angle  + (1-A)*gyro_roll_angle;
  out_roll = myIMU.roll - prev_roll;
  prev_roll = myIMU.roll;
  
  myIMU.pitch = A*accel_pitch_angle + (1-A)*gyro_pitch_angle;
  out_pitch = myIMU.pitch - prev_pitch;
  prev_pitch = myIMU.pitch;
  
  myIMU.yaw   = gyro_yaw_angle;
  out_yaw = myIMU.yaw - prev_yaw;
  prev_yaw = myIMU.yaw;
    
  

  //Debug Output
//  Serial.print("AcX = "); Serial.print(myIMU.ax);
//  Serial.print("\t| AcY = "); Serial.print(myIMU.ay);
//  Serial.print("\t| AcZ = "); Serial.print(myIMU.az);
//  Serial.print("\t| Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//  Serial.print("\t| GyX = "); Serial.print(myIMU.gx);
//  Serial.print("\t| GyY = "); Serial.print(myIMU.gy);
//  Serial.print("\t| GyZ = "); Serial.println(myIMU.gz);
//  Serial.print("\t| mx = "); Serial.print(myIMU.mx);
//  Serial.print("\t| my = "); Serial.print(myIMU.my);
//  Serial.print("\t| mz = "); Serial.print(myIMU.mz);
//  Serial.print("\t| Magnitude = "); Serial.println(magnitude);
//  Serial.print(gyro_roll_angle); Serial.print(","); Serial.print(accel_roll_angle); Serial.print(","); Serial.println(myIMU.roll);
//  Serial.print(gyro_pitch_angle); Serial.print(","); Serial.print(accel_pitch_angle); Serial.print(","); Serial.println(myIMU.pitch);

  //Final Output
  Serial.print(myIMU.mx); Serial.print(",");
  Serial.print(myIMU.my); Serial.print(",");
  Serial.print(myIMU.mz); Serial.print(",");
  Serial.print(magnitude); Serial.print(",");
  Serial.print(myIMU.roll); Serial.print("\t");
//  Serial.print(out_roll); Serial.print("\t");
  Serial.print(myIMU.pitch); Serial.print("\t");
//  Serial.print(out_pitch); Serial.print("\t");
  Serial.print(myIMU.yaw);  Serial.print("\n");
//  Serial.print(out_yaw);  Serial.print("\n");
  delay(333);   //TODO this is probably not necessary. Evaluate.


  //Prototype Output
//  char msg[50] = "";
//  sprintf(msg,"%d,%d,%d,%d\n",myIMU.mx*100, myIMU.my*100, myIMU.mz*100, magnitude*100);
//  Serial.print(msg);
//  dtostrf(myIMU.mx,5,2,msg);
//  Serial.print(msg);
  

  //Motor Command Section
  if(magnitude > 30000)
  {
    HMD.Waveform(0, 13);
    HMD.go();
    delay(300);
  }
  else if(magnitude > 20000)
  {
    HMD.Waveform(0, 13);                                                                      //loads wave into sequence register 0+seq
    HMD.go();                                                                                 //Plays the sequence registers.
    delay(600);                                                                               //Without a delay, the motor quickly stops vibrating. Possibly a safety check or race condition?
                                                                                              //Of course, leaving this in means sending updates less frequently, so a solution will be needed. Set and check timer?
  }

//  if(magnitude > 30000)
//  {
//    if(millis() > vibe_timer + 300)
//    {
//      vibe_timer = millis();
//      HMD.Waveform(0, 13);
//      HMD.go();
//    }
//  }
//  else if(magnitude > 20000)
//  {
//    if(millis() > vibe_timer + 600)
//    {
//      vibe_timer = millis();                                                                //loads wave into sequence register 0+seq
//      HMD.Waveform(0, 13);                                                                  //Plays the sequence registers.
//      HMD.go();                                                                             //Without a delay, the motor quickly stops vibrating. Possibly a safety check or race condition.
//    }
//  }
}

void magcalMPU9250(float * dest1, float * dest2) //hand in bias and Scale
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
    // shoot for ~fifteen seconds of mag data
    if(myIMU.readModeData() == 0x02) sample_count = 128;                                      // at 8 Hz ODR, new mag data is available every 125 ms
    if(myIMU.readModeData() == 0x06) sample_count = 1500;                                     // at 100 Hz ODR, new mag data is available every 10 ms
   for(ii = 0; ii < sample_count; ii++) {
    myIMU.readMagData(mag_temp);                                                              // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if(myIMU.readModeData() == 0x02) delay(135);                                              // at 8 Hz ODR, new mag data is available every 125 ms
    if(myIMU.readModeData() == 0x06) delay(12);                                               // at 100 Hz ODR, new mag data is available every 10 ms
    }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction to co-locate the centers of each reading plane.
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;                                               // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;                                               // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;                                               // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*myIMU.mRes*myIMU.magCalibration[0];                        // save mag biases in G (not mG?) for main program
    dest1[1] = (float) mag_bias[1]*myIMU.mRes*myIMU.magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*myIMU.mRes*myIMU.magCalibration[2];  
       
    // Get soft iron correction estimate to normalize the sensor readings to an ellipsoid reading surface
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
