/*
 * based on the GyRobo Code "Bal_Code_v8_with_bluetooth_working_SVPWM"
 * and Gyro-2016-04-24A 
 * 
 * Still not balancing - just reading angle and acceleration and calculating angle
 * 
 * HC-05 Bluetooth setup for 19200 baud on the TX and RX pins
 * Joystick BT Commander Version 5.2 
 * Button #1: "ON", Button #2: "Fast"
 * 
 * Data field #1: angleY, Data field #2: L_speed, Data field #3: R_speed
 * note: cycle time must not exceed 4ms
 *
 * Gyro-2016-04-24A isbased on the MPU6050.ino code and 
 * the I2C.ino file in the MPU6050 example of the kalmanFilter library  
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


#pragma GCC optimize ("3")

#include <Wire.h>

//main loop--------------------------------------------------------
#define LEDPIN   13

bool power = false;            //run only when power is true

uint16_t freqCounter = 0;
uint16_t oldfreqCounter = 0;
uint16_t loop_time = 0;         //how fast is the main loop running


//Bluetooth input stuff------------------------------------------
#define    STX          0x02	// debut d'ordre
#define    ETX          0x03	// fin d'ordre
#define    SLOW         750                             // Datafields refresh rate (ms)
#define    FAST         250                             // Datafields refresh rate (ms)

int16_t joyY, joyX;                                         //smartphone joystick input values

uint8_t cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
uint8_t buttonStatus = 0;                                  // first Byte sent to Android device
long previousMillis = 0;                                // will store last time Buttons status was updated
long sendInterval = SLOW;                               // interval between Buttons status transmission (milliseconds)

//motor control-------------------------------------------------------

#define MAX_POWER   150   // up to 255

// moteur de gauche
int motorG_enable = 10; //pwm sur timer1
int motorG_forward = 13;
int motorG_backward = 12;

// moteur de droite 
int motorD_enable = 9; //pwm sur timer1
int motorD_forward = 7;
int motorD_backward = 8;

//motor numbers
#define L_Motor 0
#define R_Motor 1

uint16_t R_MotorStep = 0;
uint16_t L_MotorStep = 0;

uint16_t MotorPower = 120; // 0 to 255, 255==100% power

uint8_t idl_motor_power = 80;

uint8_t resp_rate = 1; //bluetooth remote joystick multiplier

bool Robot_Direction;

//rotation speed for turning
int8_t rot_Speed = 0;

float robot_speed;

// speed of motors, 0 to 255
int16_t R_Speed = 0;
int16_t L_Speed = 0;

//PID stuff---------------------------------------------------------------
int32_t robot_position = 0; //offset from corect position


//loop tuning values
#define kc 0.0002           //position feedback gain, (reposition robot after disturbance)0.0002
#define kv 0.02             //velocity feedback gain 0.02
#define kp 0.2              //angle gain 0.2
#define kd 0.025/(2*0.004) //angle velocity gain 0.025

#define PID_out_max 100
#define PID_out_min -100

float PID_in, last_PID_in, Last_last_PID_in, PID_out;

//Gyro MPU-6050 stuff---------------------------------------------------------------
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


/****************************************************************************************************
 * SETUP
 ****************************************************************************************************/
 
void setup()
{

  // empecher les moteurs de demarrer durant le setup
  pinMode (9, OUTPUT);
  pinMode (10, OUTPUT);
  digitalWrite(9, 0);
  digitalWrite(10, 0);
  // FIN empecher les moteurs de demarrer durant le setup
  
  //Setup MOTOR Controller
  Bl_Setup();

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

  Serial.println("Setup termine"); 

  // empty RX buffer
  while (Serial.available())  Serial.read();        
}

/****************************************************************************************************
 * LOOP
 ****************************************************************************************************/
 
void loop()
{

  //send bluetooth output to smartphone every ~0.13s    
  sendBlueToothData();

  //receive bluetooth input from smartphone
  bluetooth();

  //Read the raw data from the gyroscope
  read_angle_data();

  // convert the raw data with the complementary filter
  convert_raw_data();

  //run if on sent from smartphone and battery is charged
  if (power == true ) PID_and_motor_comd();
  else
  {
    motorPowerOff();               //turn motors off if battery is discharged / powered off
    Last_last_PID_in = 0;
    last_PID_in = 0;
    MotorPower = 0;
    robot_speed = 0;
    robot_position = 0;
  }

  //calculate loop time
  if (freqCounter > oldfreqCounter)
  {
    if (loop_time < (freqCounter - oldfreqCounter)) loop_time = freqCounter - oldfreqCounter;
  }
}

//-----------------------------------------------------------------------------------
//bluetooth between smartphone and robot---------------------------------------------
//-----------------------------------------------------------------------------------

void bluetooth()
{
  if (Serial.available())   // data received from smartphone
  {
    cmd[0] =  Serial.read();
    if (cmd[0] == STX)
    {
      int i = 1;
      while (Serial.available())
      {
        cmd[i] = Serial.read();
        if (cmd[i] > 127 || i > 7)                   break; // Communication error
        if ((cmd[i] == ETX) && (i == 2 || i == 7))   break; // Button or Joystick data
        i++;
      }
      if      (i == 2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
      else if (i == 7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
    }
  }
}


void sendBlueToothData()
{

  static long previousMillis = 0;                             
  long currentMillis = millis();
  if(currentMillis - previousMillis > sendInterval) {   // send data back to smartphone

    loop_time = currentMillis - previousMillis;    
    previousMillis = currentMillis;
    
  // Data frame transmitted back from Arduino to Android device:
  // < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >
  // < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

  Serial.print((char)STX);                                             // Start of Transmission
  Serial.print(getButtonStatusString());    Serial.print((char)0x1);   // buttons status feedback
  Serial.print(loop_time);     Serial.print((char)0x4);                     // datafield #1
  Serial.print(L_Speed);        Serial.print((char)0x5);    // datafield #2
  Serial.print(R_Speed);                                    // datafield #3
  Serial.print((char)ETX);                                             // End of Transmission


  }
}


String getButtonStatusString()  {
  String bStatus = "";
  for (int i = 0; i < 6; i++)  {
    if (buttonStatus & (B100000 >> i))      bStatus += "1";
    else                                    bStatus += "0";
  }
  return bStatus;
}


void getJoystickState(byte data[8])    {
  joyX = (data[1] - 48) * 100 + (data[2] - 48) * 10 + (data[3] - 48); // obtain the Int from the ASCII representation
  joyY = (data[4] - 48) * 100 + (data[5] - 48) * 10 + (data[6] - 48);
  joyX = joyX - 200;                                                  // Offset to avoid
  joyY = joyY - 200;                                                  // transmitting negative numbers

  if (joyX < -100 || joyX > 100 || joyY < -100 || joyY > 100)     return; // commmunication error
}


void getButtonState(int bStatus)  {
  switch (bStatus) {
    // -----------------  BUTTON #1  -----------------------
    case 'A':
      buttonStatus |= B000001;        // ON - les autres boutons ne sont pas touchés (1)

      power = true;                   // ON

      break;
    case 'B':
      buttonStatus &= B111110;        // OFF - les autres boutons ne sont pas touchés (1)

      power = false;                   // OFF

      break;

    // -----------------  BUTTON #2  -----------------------
    case 'C':
      buttonStatus |= B000010;        // ON

      resp_rate = 2;

      break;
    case 'D':
      buttonStatus &= B111101;        // OFF
      resp_rate = 1;

      break;

/*
      
        // -----------------  BUTTON #3  -----------------------
         case 'E':
          //buttonStatus |= B000100;        // ON
           idl_motor_power += 5;

           break;
         case 'F':
           buttonStatus &= B111011;      // OFF
           // your code...

           break;

        // -----------------  BUTTON #4  -----------------------
         case 'G':
          //buttonStatus |= B001000;       // ON
           idl_motor_power -= 5;

           break;
         case 'H':
          buttonStatus &= B110111;    // OFF
           // your code...

          break;

        // -----------------  BUTTON #5  -----------------------
         case 'I':           // configured as momentary button
          buttonStatus |= B010000;        // ON
           // your code...

           break;
         case 'J':
          buttonStatus &= B101111;        // OFF
           // your code...

           break;

        // -----------------  BUTTON #6  -----------------------
         case 'K':
          buttonStatus |= B100000;        // ON
           // your code...

          break;
         case 'L':
          buttonStatus &= B011111;        // OFF
           // your code...

           break;
      */
  }
}

//--------------------------------------------------------------------------------------
//angle PID calculation and motor command signal generation-----------------------------
//--------------------------------------------------------------------------------------
void PID_and_motor_comd()
{
    //adjust motors to turn robot
    MotorPower = map(abs(joyY), 0, 100, 0, MAX_POWER);  

    if (MotorPower > MAX_POWER) MotorPower = MAX_POWER;

    R_Speed = MotorPower;
    L_Speed = MotorPower;

    //adjust motors to turn robot
    rot_Speed = - 0.3 * joyX;
    R_Speed += rot_Speed;
    L_Speed -= rot_Speed;

    if(joyY>0) {
      Robot_Direction = true ; // will go forward
    }else {
      Robot_Direction = false ; // will go backward
    }
    
    //run motors
    MoveMotors(R_Motor, Robot_Direction, R_Speed); // always forward (true)
    MoveMotors(L_Motor, Robot_Direction, L_Speed); // always forward (true)
}
  

//-----------------------------------------------------------------------------------
// motor control ---------------------------------------------
//-----------------------------------------------------------------------------------

void Bl_Setup()
{

  //on initialise les pins du moteur 1
  pinMode(motorG_forward, OUTPUT);
  pinMode(motorG_backward, OUTPUT);
  pinMode(motorG_enable, OUTPUT);
 
  //on initialise les pins du moteur 2
  pinMode(motorD_forward, OUTPUT);
  pinMode(motorD_backward, OUTPUT);
  pinMode(motorD_enable, OUTPUT);

  digitalWrite(LEDPIN, HIGH);

  // switch off PWM Power
  motorPowerOff();
}

//Fonction qui arrête / démarre les 2 moteurs 
void motorPowerOff() {

  MoveMotors(L_Motor, 0, 0);
  MoveMotors(R_Motor, 0, 0);
}

void MoveMotors(uint8_t motorNumber, bool dir, uint16_t power)
{


  // set motor pwm variables
  if (motorNumber == 0)
  {
    analogWrite(motorD_enable, power);
    digitalWrite(motorD_forward, dir);
    digitalWrite(motorD_backward, !dir);
  }

  if (motorNumber == 1)
  {
    analogWrite(motorG_enable, power);
    digitalWrite(motorG_forward, dir);
    digitalWrite(motorG_backward, !dir);
  }
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



//--------------------------------------------------------------
// code loop timing---------------------------------------------
//--------------------------------------------------------------
// minimize interrupt code length
// is called every 31.875us (510 clock cycles)  ???????
ISR( TIMER1_OVF_vect )
{
  //every 32 count of freqCounter ~1ms
  freqCounter++;

  // overflow to loop after >>8 shift
  // en principe sert seulement pour les moteurs brushless
  if ((freqCounter & 0x01) == 0)
  {
    
    R_MotorStep += R_Speed;
    L_MotorStep += L_Speed;
    
  }
}






