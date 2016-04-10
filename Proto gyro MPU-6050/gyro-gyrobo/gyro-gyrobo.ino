/****************************************
 * test gyro
 * Seulement le code lié à la lecture du gyro  
 * issu du code du balancing robot
 * voir http://www.instructables.com/id/Brushless-Gimbal-Balancing-Robot/?ALLSTEPS
 */

#define LEDPIN   13

//angle calculations from MPU-6050--------------------------------------------------
#include <Wire.h>

#define MPU6050 0x68              //Device address (standard)
#define ACCEL_CONFIG 0x1C         //Accelerometer configuration address
#define GYRO_CONFIG 0x1B          //Gyro configuration address

//Registers: Accelerometer, Temp, Gyroscope
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C


//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s


int16_t  AcX, AcY, GyZ;       // acceleration en x et y, angle en z

//IMU offset values
int16_t  AcX_offset = 0;
int16_t  AcY_offset = 0;
int16_t  GyZ_offset = 0;
int32_t  GyZ_offset_sum = 0;


void setup() {
  
  pinMode (LEDPIN, OUTPUT);
  // Start Serial Port
  Serial.begin(57600);
  Serial.println("gyro test" );
  Serial.println("Pret !");

  for (int i = 0; i < 50; i++)
  {
    digitalWrite(LEDPIN, !digitalRead(LEDPIN));
    delay (100);
  }

  //setup robot angle calculation
  Serial.println("calibrating gyroscope...........");
  angle_setup();    
}

void loop() {
  
    //calculate angle of robot
    angle_calc();

    // ajouté pour faciliter la compréhension des mouvements et leurs résultats
    delay(1000);
    Serial.println("--------------- Nouvelle Mesure ---------------");
    

}

//------------------------------------------------------------------
//Robot angle calculations------------------------------------------
//------------------------------------------------------------------

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}



//setup MPU6050
void angle_setup()
{
  Wire.begin();
  delay (100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  Serial.println("angle setup done !");
}


//calculate robot tilt angle
void angle_calc()
{
  // read raw accel/gyro measurements from device
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);  // request a total of 4 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  // request a total of 2 registers
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

/***********************
 * to print the values in a table
***********************/

  Serial.print("AcX = "); Serial.print(AcX); Serial.print("\t");
  Serial.print(" | AcY = "); Serial.print(AcY); Serial.print("\t");
  Serial.print(" | GyZ = "); Serial.print(GyZ); Serial.print("\t");
  Serial.print("\n");
/************************/

}





