/*
 * based on the GyRobo Code "Bal_Code_v8_with_bluetooth_working_SVPWM"
 * No balancing or MPU6050 accelerometer sensor 
 * 
 * Lights or flashes leds  

 * 
 * HC-05 or HC-06 Bluetooth setup for 57 600 baud on the TX and RX pins
 * Joystick BT Commander Version 5.2 
 * Button #1: " ON / OFF", 
 * Button #2: Rouge
 * Button #3: Vert
 * Button #4: Bleu
 * Data field #1: "Bat. V", Data field #2: "Power", Data field #3: "Cycle (ms)" 
 * note: cycle time must not exceed 4ms
 */


#pragma GCC optimize ("3")


//main loop--------------------------------------------------------
#define LEDPIN   13             // led intégrée à l'arduino

bool power = false;            //run only when power is true

uint16_t freqCounter = 0;
uint16_t oldfreqCounter = 0;
uint16_t loop_time = 0;         //how fast is the main loop running

String data1 = "data1";   
String data2 = "data2"; 

int color ;                   // led# to command  
bool onOff ;                  // on (1) or off (0)

//Bluetooth input stuff------------------------------------------
#define    STX          0x02	// debut d'ordre
#define    ETX          0x03	// fin d'ordre

int16_t joyY, joyX;                                         //smartphone joystick input values

uint8_t cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
uint8_t buttonStatus = 0;                                  // first Byte sent to Android device



//LED control-------------------------------------------------------

// les 4 leds
int led[4] ={11, 10, 9, 8}; ;  // 4 leds from 1 to 4
// red / blue / green / white


void setup()
{
  for (int i = 0; i < 4; i++)
  {
    pinMode (led[i], OUTPUT);
  } 
  // Start Serial Port
  Serial.begin(57600);
  Serial.println("LED_with_bluetooth_working_V0.1" );
  Serial.println("Pret !");

  for (int i = 0; i < 20; i++)
  {
    digitalWrite(led[1], !digitalRead(led[1]));
    digitalWrite(led[2], !digitalRead(led[2]));
    digitalWrite(led[3], !digitalRead(led[3]));
    digitalWrite(led[0], !digitalRead(led[4]));
    delay (100);
  }

  //Setup LED Controller

  Serial.println("Setup is almost done \n\r"); 
  LED_Setup();
  Serial.println("Setup is DONE \n\r"); 

  for (int i = 0; i < 20; i++)
  {
    for (int j = 0; j < 2; j++) {
      LED_command(j, !digitalRead(led[j])) ;
      delay (400);      
    }
    for (int j = 2; j < 4; j++) {
      LED_command(j, !digitalRead(led[j])) ;
      delay (400);      
    }
  }
  
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
    if (power == true ) {
      LED_command( color, onOff);   // will run the led command and return data2 content
    }
    else
    {
      StopLeds();               //turn all leds off
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
  Serial.print(data1);     Serial.print((char)0x4);                     // datafield #1
  Serial.print(data2);        Serial.print((char)0x5);                  // datafield #2
  Serial.print(loop_time * 0.0318);                                    // datafield #3
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
      buttonStatus |= B000001;        // ON 

      power = true;                   // ON
      data1 = "ON";
      break;
    case 'B':
      buttonStatus = B000000;        // OFF et met à 0 tous les autres boutons - pas &= B111110
      power = false;                   // OFF
      StopLeds();
      color = 4;                  // white will power on when next button true pushed
      onOff = true;               // ON
      data1 = "OFF";

      break;

    // -----------------  BUTTON #2  -----------------------
    case 'C':
      buttonStatus |= B000010;        

      color = 0;                  // red
      onOff = true;               // ON
      data2 = "Red ON";
      break;
    case 'D':
      buttonStatus &= B111101;        // tourne sens inverse aiguille montre
      color = 0;                  // red
      onOff = false;               // OFF
      data2 = "Red OFF";
      break;

      
   // -----------------  BUTTON #3  -----------------------
   case 'E':
      buttonStatus |= B000100;        // avance

    color = 1;                  // blue
    onOff = true;               // ON         
    data2 = "blue ON";
     break;
   case 'F':
      buttonStatus &= B111011;      // OFF
      color = 1;                  // blue
      onOff = false;               // OFF
      data2 = "blue OFF";
      break;

    // -----------------  BUTTON #4  -----------------------
    case 'G':
      buttonStatus |= B001000;       // recule

      color = 2;                  // green
      onOff = true;               // ON   
      data2 = "green ON";
      break;
     case 'H':
      buttonStatus &= B110111;    // OFF
      color = 2;                  // green
      onOff = false;               // OFF
      data2 = "green OFF";
      break;
/*
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
    // R_MotorStep += R_Speed;
    // L_MotorStep += L_Speed;
  }
}


//--------------------------------------------------------------------
// LED SETUP-------------------------------------------------
//--------------------------------------------------------------------
void LED_Setup()
{

  for (int i = 0; i < 4; i++)
  {
    digitalWrite (led[i], HIGH);
  }

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

  // switch off LEDS
  StopLeds();
}



//-----------------------------------------------------------------------------------
// LED control ---------------------------------------------
//-----------------------------------------------------------------------------------

// allumer une des led
void LED_command(int num, bool pos) {

    digitalWrite(led[num], pos);
}


//arrêter toutes les led  
void StopLeds() {

  for (int i = 0; i < 4; i++)
  {
    digitalWrite(led[i], LOW);
  }
 

}






