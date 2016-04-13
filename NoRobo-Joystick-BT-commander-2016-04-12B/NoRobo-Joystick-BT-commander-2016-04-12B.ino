/*
 * based on the GyRobo Code "Bal_Code_v8_with_bluetooth_working_SVPWM"
 * No balancing 
 *  * Gyro MPU6050 on the NoRobo but it's used only to get data, not to change anything
 * 
 * This is a mix of NoRobo-Joystick-BT-commander-2016-04-07.ino & the balancing GyRobo code
 * Still not balancing - just reading angle and acceleration and calculating angle
 * 
 * Reste 1 défaut : au démarrage, le moteur D fait des à coups
 * 
 * HC-05 Bluetooth setup for 57 600 baud on the TX and RX pins
 * Joystick BT Commander Version 5.2 
 * Button #1: "ON", Button #2: "Fast"
 * 
 * Data field #1: "data1", Data field #2: "Power", Data field #3: "Cycle (ms)" 
 * note: cycle time must not exceed 4ms
 */


#pragma GCC optimize ("3")


//main loop--------------------------------------------------------
#define LEDPIN   13

bool power = false;            //run only when power is true

uint16_t freqCounter = 0;
uint16_t oldfreqCounter = 0;
uint16_t loop_time = 0;         //how fast is the main loop running


//Bluetooth input stuff------------------------------------------
#define    STX          0x02	// debut d'ordre
#define    ETX          0x03	// fin d'ordre

int16_t joyY, joyX;                                         //smartphone joystick input values

uint8_t cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
uint8_t buttonStatus = 0;                                  // first Byte sent to Android device


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


int16_t  AcZ, AcX, GyY;       // my gyro with x forward, y left-right, z vertical

int16_t  AcZ_offset = 0;
int16_t  AcX_offset = 0;
int16_t  GyY_offset = 0;
int32_t  GyY_offset_sum = 0;

int32_t GyY_filter[32];
uint8_t filter_count = 0;
int32_t  GyY_F;
float robot_angle;
float Acc_angle;            //angle calculated from acc. measurments

bool vertical = false;      //is the robot vertical enough to run

bool GyY_filter_on = true;  //apply simple average filter to Z gyro reading

#define Gyro_amount 0.996   //percent of gyro in complementary filter T/(T+del_t) del_t: sampling rate, T acc. timeconstant ~1s




//motor control-------------------------------------------------------

#define MAX_POWER   150   // up to 255

// moteur de gauche
int motorG_enable = 11; //pwm
int motorG_forward = 13;
int motorG_backward = 12;

// moteur de droite 
int motorD_enable = 10; //pwm
int motorD_forward = 8;
int motorD_backward = 9;



//motor numbers
#define L_Motor 0
#define R_Motor 1

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


/****************************************************************************************************
 * SETUP
 ****************************************************************************************************/
 
void setup()
{
  pinMode (LEDPIN, OUTPUT);
 
  // Start Serial Port
  Serial.begin(57600);
  Serial.println("NoRobo_with_bluetooth_working_V0.1" );
  Serial.println("The NoRobo should be on its head, (wheels up !)" );
  Serial.println("place robot so that MPU6050 is horizontal withing 5sec. of power for gyroscope calibration");

  for (int i = 0; i < 50; i++)
  {
    digitalWrite(LEDPIN, !digitalRead(LEDPIN));
    delay (100);
  }

  //setup robot angle calculation
  Serial.println("calibrating gyroscope...........");
  angle_setup();  
  Serial.println("GYRO Setup is DONE"); 

  //Setup MOTOR Controller

  Bl_Setup();
  Serial.println("place robot vertical to run"); 

  // empty RX buffer
  while (Serial.available())  Serial.read();        
}

/****************************************************************************************************
 * LOOP
 ****************************************************************************************************/
 
void loop()
{
  //run main loop every ~4ms
  if ((freqCounter & 0x07f) == 0)
  {
    // record when loop starts
    oldfreqCounter = freqCounter;

    //test battery voltage every ~1s
    if ((freqCounter & 0x7FFF) == 0)
    {
      // battery_test();
    }

    //send bluetooth output to smartphone every ~0.13s
    if ((freqCounter & 0xFFF) == 0)
    {
      //sendBlueToothData();
    }

    //receive bluetooth input from smartphone
    //bluetooth();

    //calculate angle of robot
    angle_calc();

    /***********************
     * to print the values in a table
    ***********************/
    
      Serial.print("GyY = "); Serial.print(GyY); Serial.print("\t");
      Serial.print(" | AcX = "); Serial.print(AcX); Serial.print("\t");
      Serial.print(" | AcZ = "); Serial.print(AcZ); Serial.print("\t");
      Serial.print(" | robot_angle = "); Serial.print(robot_angle); Serial.print("\t"); 
      Serial.print(" | Acc_angle = "); Serial.print(Acc_angle); Serial.print("\t");
      // Serial.print(" | vertical = "); Serial.print(vertical); Serial.print("\t");
      Serial.print("\n");
      delay(10000);
    /************************/

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
  // Data frame transmitted back from Arduino to Android device:
  // < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >
  // < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

  Serial.print((char)STX);                                             // Start of Transmission
  Serial.print(getButtonStatusString());    Serial.print((char)0x1);   // buttons status feedback
  Serial.print("!!!!! ");     Serial.print((char)0x4);                     // datafield #1
  Serial.print(robot_angle);        Serial.print((char)0x5);    // datafield #2
  Serial.print(vertical);                                    // datafield #3
  Serial.print((char)ETX);                                             // End of Transmission

  loop_time = 0;
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
      Robot_Direction = true ;
    }else {
      Robot_Direction = false ;
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

  cli();//stop interrupts

  //timer setup for 31.250KHZ phase correct PWM
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
  TCCR0B = _BV(CS00);
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10);
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  // enable Timer 1 interrupt
  TIMSK1 = 0;
  TIMSK1 |= _BV(TOIE1);
  // disable arduino standard timer interrupt
  TIMSK0 &= ~_BV(TOIE1);

  sei(); // Start Interrupt

  //turn off all PWM signals
  OCR2A = 0;  //11  APIN
  OCR2B = 0;  //D3
  OCR1A = 0;  //D9  CPIN
  OCR1B = 0;  //D10 BPIN
  OCR0A = 0;  //D6
  OCR0B = 0;  //D5

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

  // calc Z gyro offset by averaging 1024 values
  if (GyY_filter_on == true)
  {
    GyY_filter_on = false;
    for (int i = 0; i < 1024; i++)
    {
      angle_calc();
      Serial.println(GyY);
      GyY_offset_sum += GyY;
      digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      delay (10);
    }
    GyY_filter_on = true;
  }
  else
  {
    for (int i = 0; i < 1024; i++)
    {
      angle_calc();
      Serial.println(GyY);
      GyY_offset_sum += GyY;
      digitalWrite(LEDPIN, !digitalRead(LEDPIN));
      delay (10);
    }
  }
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyY offset value = ");
  Serial.println(GyY_offset);
}


//calculate robot tilt angle
void angle_calc()
{
  // read raw accel/gyro measurements from device
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050,2,true);  // request a total of 2 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3F);          // starting with register 0x3F (ACCEL_ZOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  // request a total of 2 registers
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x45);       // starting with register 0x45 (GYRO_YOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  // request a total of 2 registers
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)

  if (GyY_filter_on == true)
  {
    // simple low pass filter on gyro
    GyY_filter[filter_count] = GyY;

    filter_count++;

    if (filter_count > 15) filter_count = 0;

    GyY_F = 0;
    for (int i = 0; i < 16; i++)
    {
      GyY_F += GyY_filter[i];
      GyY = GyY_F >> 4;
    }
  }

  // add mpu6050 offset values
  AcZ += AcZ_offset;
  AcX += AcX_offset;
  GyY += GyY_offset;

  //use complementary filter to calculate robot angle
  robot_angle -= GyY * 6.07968E-5;                      //integrate gyroscope to get angle       * 0.003984 (sec) / 65.536 (bits / (deg/sec))
  //robot_angle += GyY * 6.07968E-5;                      //integrate gyroscope to get angle       * 0.003984 (sec) / 65.536 (bits / (deg/sec))
  Acc_angle =  atan2(AcX, -AcZ) * 57.2958;              //angle from acc. values       * 57.2958 (deg/rad)
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);

  
  //check if robot is vertical
  if (robot_angle > 50 || robot_angle < -50) vertical = false;
  if (robot_angle < 1 && robot_angle > -1) vertical = true;


 
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

  if ((freqCounter & 0x01) == 0)
  {
    /*
    R_MotorStep += R_Speed;
    L_MotorStep += L_Speed;
    */
  }
}






