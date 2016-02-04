/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_pitch1 // Comment out to restrict roll1 to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

#include <math.h>

#define MPU6050_I2C_ADDRESS 0x68

#define FREQ  30.0 // sample freq in Hz

Kalman kalmanX2; // Create the Kalman instances
Kalman kalmanY2;

Kalman kalmanX1; // Create the Kalman instances
Kalman kalmanY1;

/* IMU Data */
double accX1, accY1, accZ1;
double accX2, accY2, accZ2;
double gyroX1, gyroY1, gyroZ1;
double gyroX2, gyroY2, gyroZ2;
int16_t tempRaw;

double gyroXangle1, gyroYangle1; // Angle calculate using the gyro only
double compAngleX1, compAngleY1; // Calculated angle using a complementary filter
double kalAngleX1, kalAngleY1; // Calculated angle using a Kalman filter

double gyroXangle2, gyroYangle2; // Angle calculate using the gyro only
double compAngleX2, compAngleY2; // Calculated angle using a complementary filter
double kalAngleX2, kalAngleY2; // Calculated angle using a Kalman filter

uint32_t timer;

// TODO: Make calibration routine

void setup() 
{
  uint8_t c;
  uint8_t sample_div;
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif


i2c_write_reg (0x68, 0x6b, 0x00); //done
  i2c_write_reg (0x68, 0x1a, 0x00); //temp measurement?? :O
  //i2c_write_reg(0x68, 0x1b, 0x08);
  i2c_write_reg(0x68, 0x1b, 0x00);
  i2c_write_reg(0x68, 0x1c, 0x00);
  
  sample_div = 1000 / FREQ - 1;
  i2c_write_reg (0x68, 0x19, sample_div); // ???? problem
  
  i2c_write_reg (0x69, 0x6b, 0x00);
  i2c_write_reg (0x69, 0x1a, 0x01); //temp measurement?? :O
  i2c_write_reg(0x69, 0x1b, 0x08);
  sample_div = 1000 / FREQ - 1;
  i2c_write_reg (0x69, 0x19, 0x07);
  /*
  
i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  */
  

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  
  read_sensor_data1();
  //read_sensor_data2();


   // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll1  = atan2(accY1, accZ1) * RAD_TO_DEG;
  double pitch1 = atan(-accX1 / sqrt(accY1 * accY1 + accZ1 * accZ1)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll1  = atan(accY1 / sqrt(accX1 * accX1 + accZ1 * accZ1)) * RAD_TO_DEG;
  double pitch1 = atan2(-accX1, accZ1) * RAD_TO_DEG;
#endif

  kalmanX1.setAngle(roll1); // Set starting angle
  kalmanY1.setAngle(pitch1);
  gyroXangle1 = roll1;
  gyroYangle1 = pitch1;
  compAngleX1 = roll1;
  compAngleY1 = pitch1;
  
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll2  = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll2  = atan(accY2 / sqrt(accX2 * accX2 + accZ2 * accZ2)) * RAD_TO_DEG;
  double pitch2 = atan2(-accX2, accZ2) * RAD_TO_DEG;
#endif


  
   kalmanX2.setAngle(roll2); // Set starting angle
  kalmanY2.setAngle(pitch2);
  gyroXangle2 = roll2;
  gyroYangle2 = pitch2;
  compAngleX2 = roll2;
  compAngleY2 = pitch2;

  timer = micros();
}

void loop() 
{
  /* Update all the values */
 
read_sensor_data1();

   // Calculate delta time
 
  compute_print();

   // It is then converted from radians to degrees

  delay(2);
}

int i2c_write(int addr, int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(addr);
  n = Wire.write(start);        // write the start address
  if (n != 1) // 1 should come I guess
  return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)  //n should be size I guess
  return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0) //returns eror
  return (error);

  return (0);         // return : no error
}


int i2c_write_reg(int addr, int reg, uint8_t data)
{
  int error;
  
  error = i2c_write(addr, reg, &data, 1);
  return (error);
}

void read_sensor_data1()
{
 uint8_t i2cData1[14];
 uint8_t error1;
 // read imu data
 error1 = i2c_read(0x68, 0x3b, i2cData1, 14); //why 3b? for what purpose???
 if(error1!=0)
 return;

 // assemble 16 bit sensor data
 accX1 = ((i2cData1[0] << 8) | i2cData1[1]);
 accY1 = ((i2cData1[2] << 8) | i2cData1[3]);
 accZ1 = ((i2cData1[4] << 8) | i2cData1[5]);

 gyroX1 = (((i2cData1[8] << 8) | i2cData1[9]));
 gyroY1 = (((i2cData1[10] << 8) | i2cData1[11]));
 gyroY1 = (((i2cData1[12] << 8) | i2cData1[13]));
}

int i2c_read(int addr, int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(addr);
  n = Wire.write(start); //should come 1 I guess
  if (n != 1)
  return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0) //should come 0 I guess
  return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(addr, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
  return (-11);

  return (0);  // return : no error
}

void compute_print()
{
  double dt = (double)(micros() - timer) / 1000000;
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll1  = atan2(accY1, accZ1) * RAD_TO_DEG;
  double pitch1 = atan(-accX / sqrt(accY1 * accY1 + accZ1 * accZ1)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll1  = atan(accY1 / sqrt(accX1 * accX1 + accZ1 * accZ1)) * RAD_TO_DEG;
  double pitch1 = atan2(-accX1, accZ1) * RAD_TO_DEG;
#endif

  double gyroXrate1 = gyroX1 / 131.0; // Convert to deg/s
  double gyroYrate1 = gyroY1 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll1 < -90 && kalAngleX1 > 90) || (roll1 > 90 && kalAngleX1 < -90)) {
    kalmanX1.setAngle(roll1);
    compAngleX1 = roll1;
    kalAngleX1 = roll1;
    gyroXangle1 = roll1;
  } else
    kalAngleX1 = kalmanX1.getAngle(roll1, gyroXrate1, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX1) > 90)
    gyroYrate1 = -gyroYrate1; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY1 = kalmanY1.getAngle(pitch1, gyroYrate1, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch1 < -90 && kalAngleY1 > 90) || (pitch1 > 90 && kalAngleY1 < -90)) {
    kalmanY1.setAngle(pitch1);
    compAngleY1 = pitch1;
    kalAngleY1 = pitch1;
    gyroYangle1 = pitch1;
  } else
    kalAngleY1 = kalmanY1.getAngle(pitch1, gyroYrate1, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY1) > 90)
    gyroXrate1 = -gyroXrate1; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX1 = kalmanX1.getAngle(roll1, gyroXrate1, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle1 += gyroXrate1 * dt; // Calculate gyro angle without any filter
  gyroYangle1 += gyroYrate1 * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX1 = 0.93 * (compAngleX1 + gyroXrate1 * dt) + 0.07 * roll1; // Calculate the angle using a Complimentary filter
  compAngleY1 = 0.93 * (compAngleY1 + gyroYrate1 * dt) + 0.07 * pitch1;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle1 < -180 || gyroXangle1 > 180)
    gyroXangle1 = kalAngleX1;
  if (gyroYangle1 < -180 || gyroYangle1 > 180)
    gyroYangle1 = kalAngleY1;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX1); Serial.print("\t");
  Serial.print(accY1); Serial.print("\t");
  Serial.print(accZ1); Serial.print("\t");

  Serial.print(gyroX1); Serial.print("\t");
  Serial.print(gyroY1); Serial.print("\t");
  Serial.print(gyroZ1); Serial.print("\t");

  Serial.print("\t");
#endif

 // Serial.print(roll1); Serial.print("\t");
 // Serial.print(gyroXangle); Serial.print("\t");
  //Serial.print(compAngleX1); Serial.print("\t");
  //Serial.print(kalAngleX1); Serial.print("\t");
  
  Serial.print(compAngleY1); Serial.print("\t");
  Serial.print(kalAngleY1); Serial.print("\t");

  Serial.print("\t");
  /*

  Serial.print(pitch1); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
  */

#if 0 // Set to 1 to print the temperature
  Serial.print("\t");
  //Serial.print(temperature); Serial.print("\t");
#endif

  Serial.print("\r\n");
   timer = micros();
}
