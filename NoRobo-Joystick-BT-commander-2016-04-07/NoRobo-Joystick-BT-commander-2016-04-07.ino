/*
 * based on the GyRobo Code "Bal_Code_v8_with_bluetooth_working_SVPWM"
 * No balancing or MPU6050 accelerometer sensor 
 * 
 * fonctionne correctement. 
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

uint8_t idl_motor_power = 0;

uint8_t resp_rate = 1; //bluetooth remote joystick multiplier

bool Robot_Direction;

//rotation speed for turning
int8_t rot_Speed = 0;

// speed of motors, 0 to 255
int16_t R_Speed = 0;
int16_t L_Speed = 0;

void setup()
{
  pinMode (LEDPIN, OUTPUT);
 
  // Start Serial Port
  Serial.begin(57600);
  Serial.println("NoRobo_with_bluetooth_working_V0.1" );
  Serial.println("Pret !");

  for (int i = 0; i < 50; i++)
  {
    digitalWrite(LEDPIN, !digitalRead(LEDPIN));
    delay (100);
  }

  //Setup MOTOR Controller

  Serial.println("Setup is almost done \n\r"); 
  Bl_Setup();
  Serial.println("Setup is DONE \n\r"); 

 
  // empty RX buffer
  while (Serial.available())  Serial.read();        
}


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
      sendBlueToothData();
    }

    //receive bluetooth input from smartphone
    bluetooth();

    //run if on sent from smartphone and battery is charged
    if (power == true ) PID_and_motor_comd();
    else
    {
      motorPowerOff();               //turn motors off if battery is discharged / powered off

      MotorPower = 0;
      // robot_speed = 0;

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
  Serial.print(MotorPower);     Serial.print((char)0x4);                     // datafield #1
  Serial.print(R_Speed);        Serial.print((char)0x5);    // datafield #2
  Serial.print(L_Speed);                                    // datafield #3
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




