/*
* code which works whith the hc-06 and the hc-05 
* based on demo code for android joystick bluetooth commander 
* android app is downloadable here : http://forum.arduino.cc/index.php?PHPSESSID=jfqqvuet348fipgf2pv1osj0c7&topic=173246.0
* demo code was also downloaded there
*
* attention ne fonctionne pas à 230400 baud (caractères "gibberish")
* fonctionne bien à 57600 baud ou moins (si le module bluetooth est réglé à la même vitesse)
* et si la console série aussi réglée en bas à droite à la même vitesse. 
*/

#define VERSION     "\n\joystick-bt-led-2016-03-12.ino for Joystick Bluetooth commander v5.2 app"


// Demo setup:
// Button #1 controls pin #13 LED
// button #2 : controls pin #7 White led
// Button #4 toggle datafield display rate
// Button #5 configured as "push" button (momentary)
// Other buttons display demo message

// Arduino pin#2 to TX BlueTooth module
// Arduino pin#3 to RX BlueTooth module
// make sure your BT board is set @57600 bps
// better remove SoftSerial for PWM based projects

// For Mega 2560:
// remove   #include "SoftwareSerial.h", SoftwareSerial mySerial(2,3);
// search/replace  mySerial  >> Serial1
// pin#18 to RX bluetooth module, pin#19 to TX bluetooth module

#include "SoftwareSerial.h"

#define    STX          0x02                          // start a message
#define    ETX          0x03                          // end message
#define    ledPin       13
#define    WhiteLed     7
#define    SLOW         750                            // Datafields refresh rate (ms)
#define    FAST         250                             // Datafields refresh rate (ms)

SoftwareSerial mySerial(2,3);                           // BlueTooth module: pin#2=TX pin#3=RX
byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
byte buttonStatus = 0;                                  // first Byte sent to Android device
long previousMillis = 0;                                // will store last time Buttons status was updated
long sendInterval = SLOW;                               // interval between Buttons status transmission (milliseconds)
String displayStatus = "xxxx";                          // message to Android device

void setup()  {
  Serial.begin(57600);
  mySerial.begin(57600);                                // 57600 = max value for softserial
  pinMode(ledPin, OUTPUT);   
  pinMode(WhiteLed, OUTPUT);  
  Serial.println(VERSION);
  while(mySerial.available())  mySerial.read();         // empty RX buffer
  digitalWrite(WhiteLed, LOW);                          // set off this led
  digitalWrite(ledPin, LOW);                            // set off this led
}

void loop() {
  if(mySerial.available())  {                           // data received from smartphone
    delay(2);
    cmd[0] =  mySerial.read();  
    if(cmd[0] == STX)  {
      int i=1;      
      while(mySerial.available())  {
        delay(1);
        cmd[i] = mySerial.read();
        if(cmd[i]>127 || i>7)                 break;     // Communication error
        if((cmd[i]==ETX) && (i==2 || i==7))   break;     // Button or Joystick data
        i++;
      }
      if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
      else if(i==7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
    }
  } 
  sendBlueToothData(); 
}

void sendBlueToothData()  {
  static long previousMillis = 0;                             
  long currentMillis = millis();
  if(currentMillis - previousMillis > sendInterval) {   // send data back to smartphone
    previousMillis = currentMillis; 

// Data frame transmitted back from Arduino to Android device:
// < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >  
// < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example

    mySerial.print((char)STX);                                             // Start of Transmission
    mySerial.print(getButtonStatusString());  mySerial.print((char)0x1);   // buttons status feedback
    mySerial.print(GetdataInt1());            mySerial.print((char)0x4);   // datafield #1
    mySerial.print(GetdataFloat2());          mySerial.print((char)0x5);   // datafield #2
    mySerial.print(displayStatus);                                         // datafield #3
    mySerial.print((char)ETX);                                             // End of Transmission
  }  
}

String getButtonStatusString()  {
  String bStatus = "";
  for(int i=0; i<6; i++)  {
    if(buttonStatus & (B100000 >>i))      bStatus += "1";
    else                                  bStatus += "0";
  }
  return bStatus;
}

int GetdataInt1()  {              // Data dummy values sent to Android device for demo purpose
  static int i= -30;              // Replace with your own code
  i ++;
  if(i >0)    i = -30;
  return i;  
}

float GetdataFloat2()  {           // Data dummy values sent to Android device for demo purpose
  static float i=50;               // Replace with your own code
  i-=.5;
  if(i <-50)    i = 50;
  return i;  
}

void getJoystickState(byte data[8])    {
  int joyX = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48);       // obtain the Int from the ASCII representation
  int joyY = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
  joyX = joyX - 200;                                                  // Offset to avoid
  joyY = joyY - 200;                                                  // transmitting negative numbers

  if(joyX<-100 || joyX>100 || joyY<-100 || joyY>100)     return;      // commmunication error
  
// Your code here ...
    Serial.print("Joystick position:  ");
    Serial.print(joyX);  
    Serial.print(", ");  
    Serial.println(joyY); 
}

void getButtonState(int bStatus)  {
  switch (bStatus) {
// -----------------  BUTTON #1  -----------------------
    case 'A':
      buttonStatus |= B000001;        // ON
      Serial.println("\n** Button_1: ON **");
      // your code...      
	  displayStatus = "LED #13 <ON>";
      Serial.println(displayStatus);
      digitalWrite(ledPin, HIGH);
      break;
    case 'B':
      buttonStatus &= B111110;        // OFF
      Serial.println("\n** Button_1: OFF **");
      // your code...      
	    displayStatus = "LED #13 <OFF>";
      Serial.println(displayStatus);
      digitalWrite(ledPin, LOW);
      break;

// -----------------  BUTTON #2  -----------------------
    case 'C':
      buttonStatus |= B000010;        // ON
      Serial.println("\n** Button_2: ON **");
      // your code...      
	    displayStatus = "white LED <ON>";
      Serial.println(displayStatus);
      digitalWrite(WhiteLed, HIGH);
      break;
    case 'D':
      buttonStatus &= B111101;        // OFF
      Serial.println("\n** Button_2: OFF **");
      // your code...      
	    displayStatus = "white LED <OFF>";
      Serial.println(displayStatus);
      digitalWrite(WhiteLed, LOW);
      break;

// -----------------  BUTTON #3  -----------------------
    case 'E':
      buttonStatus |= B000100;        // ON
      Serial.println("\n** Button_3: ON **");
      // your code...      
      displayStatus = "Motor #1 enabled"; // Demo text message
      Serial.println(displayStatus);
      break;
    case 'F':
      buttonStatus &= B111011;      // OFF
      Serial.println("\n** Button_3: OFF **");
      // your code...      
      displayStatus = "Motor #1 stopped";
      Serial.println(displayStatus);
      break;

// -----------------  BUTTON #4  -----------------------
    case 'G':
      buttonStatus |= B001000;       // ON
      Serial.println("\n** Button_4: ON **");
      // your code...      
      displayStatus = "Datafield update <FAST>";
      Serial.println(displayStatus);
      sendInterval = FAST;
      break;
    case 'H':
      buttonStatus &= B110111;    // OFF
      Serial.println("\n** Button_4: OFF **");
      // your code...      
      displayStatus = "Datafield update <SLOW>";
      Serial.println(displayStatus);
      sendInterval = SLOW;
     break;

// -----------------  BUTTON #5  -----------------------
    case 'I':           // configured as momentary button
//      buttonStatus |= B010000;        // ON
      Serial.println("\n** Button_5: ++ pushed ++ **");
      // your code...      
      displayStatus = "Button5: <pushed>";
      break;
//   case 'J':
//     buttonStatus &= B101111;        // OFF
//     // your code...      
//     break;

// -----------------  BUTTON #6  -----------------------
    case 'K':
      buttonStatus |= B100000;        // ON
      Serial.println("\n** Button_6: ON **");
      // your code...      
       displayStatus = "Button6 <ON>"; // Demo text message
     break;
    case 'L':
      buttonStatus &= B011111;        // OFF
      Serial.println("\n** Button_6: OFF **");
      // your code...      
      displayStatus = "Button6 <OFF>";
      break;
  }
// ---------------------------------------------------------------
}

