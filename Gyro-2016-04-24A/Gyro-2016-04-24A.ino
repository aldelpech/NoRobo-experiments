/*
 * based on the MPU6050.ino code and the I2C.ino file in the MPU6050 example of the kalmanFilter library  
 * https://github.com/TKJElectronics/KalmanFilter
 * and the GeekMum's code (which uses the complementary filter)
 * it does not use the kalman filter library itself
 * 
 *  !!!! pour ne pas avoir erreur i2cwrite was not declared !!!!
 *  dans le répertoire MPU6050 de la librairie kalman,
 *  copier le fichier I2C.ino et le placer dans 
 *  le répertoire où est mis ce fichier. 
 *  Normalement en fermant puis ouvrant l'IDE arduino, ça fonctionne
 *  On voit le fichier I2C dans un deuxième onglet de l'IDE arduino
 * 
 */

#include <Wire.h>

/* IMU Raw Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter

uint32_t timer;

uint8_t i2cData[14]; // Buffer for I2C data

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;  
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

//  Use following variables to calibrate the acceleration sensor
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;
float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;


void setup() {  

  Serial.begin(57600);
  Wire.begin();
#if ARDUINO >= 157  // depends version of the arduino IDE, < 1.5.7 or >=
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  calibrate_sensors();

  /* Set starting measurements */
  read_angle_data();
}

void loop() {

  //Read the raw data
  read_angle_data();
  
  convert_raw_data();

}


// The sensor should be motionless on a horizontal surface 
//  while calibration is happening
void calibrate_sensors() {
  int                   num_readings = 100;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  
  //Serial.println("Starting Calibration");

  // Discard the first set of values read from the IMU
  read_angle_data();
  
  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    read_angle_data();

  // Send the data to the serial port
  Serial.print("ACC : ");
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");
  Serial.print("GYR : ");
  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.println("\t");

  }
 
  x_accel = accX / num_readings;
  y_accel = accY / num_readings;
  z_accel = accZ / num_readings;
  x_gyro = gyroX / num_readings;
  y_gyro = gyroY / num_readings;
  z_gyro = gyroZ / num_readings;
  
  // Store the raw calibration values globally
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
 
  Serial.println("Finishing Calibration");
}


void read_angle_data()
{

  last_read_time = timer;
  
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  timer = millis();
}

void convert_raw_data(){

  // Convert gyro values to degrees/sec
  float FS_SEL = 131;

  // valeur corrigée suite calibrage
  float gyro_x = (gyroX - base_x_gyro)/FS_SEL;
  float gyro_y = (gyroX - base_y_gyro)/FS_SEL;
  float gyro_z = (gyroX - base_z_gyro)/FS_SEL;

  // on utilise les données brutes d'accélaration
  float accel_x = accX;
  float accel_y = accY;
  float accel_z = accZ;

  // Get angle values from accelerometer
  float RADIANS_TO_DEGREES = 180/3.14159;
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;

  float accel_angle_z = 0;

  // time between two measurements
  float dt =(timer - last_read_time)/1000.0;
  
  // Compute the (filtered) gyro angles
  float gyro_angle_x = gyro_x*dt + last_x_angle;
  float gyro_angle_y = gyro_y*dt + last_y_angle;
  float gyro_angle_z = gyro_z*dt + last_z_angle;
  
  // Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyro_x*dt + last_gyro_x_angle;
  float unfiltered_gyro_angle_y = gyro_y*dt + last_gyro_y_angle;
  float unfiltered_gyro_angle_z = gyro_z*dt + last_gyro_z_angle;

  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.986;
  float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
  float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
  float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

  // Update the saved data with the latest values
  last_read_time = timer;
  last_x_angle = angle_x;
  last_y_angle = angle_y;
  last_z_angle = angle_z;
  last_gyro_x_angle = unfiltered_gyro_angle_x;
  last_gyro_y_angle = unfiltered_gyro_angle_y;
  last_gyro_z_angle = unfiltered_gyro_angle_z;  

  // Send the data to the serial port
  Serial.print(F("DEL:"));              //Delta T
  Serial.print(dt, DEC);
  Serial.print(F("#ACC:"));              //Accelerometer angle
  Serial.print(accel_angle_x, 2);
  Serial.print(F(","));
  Serial.print(accel_angle_y, 2);
  Serial.print(F(","));
  Serial.print(accel_angle_z, 2);
  Serial.print(F("#GYR:"));
  Serial.print(unfiltered_gyro_angle_x, 2);        //Gyroscope angle
  Serial.print(F(","));
  Serial.print(unfiltered_gyro_angle_y, 2);
  Serial.print(F(","));
  Serial.print(unfiltered_gyro_angle_z, 2);
  Serial.print(F("#FIL:"));             //Filtered angle
  Serial.print(angle_x, 2);
  Serial.print(F(","));
  Serial.print(angle_y, 2);
  Serial.print(F(","));
  Serial.print(angle_z, 2);
  Serial.println(F(""));  
}






