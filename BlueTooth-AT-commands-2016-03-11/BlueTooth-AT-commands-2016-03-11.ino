/*
* code from http://tiptopboards.free.fr/arduino_forum/viewtopic.php?f=2&t=57
* 
* cablage 
* VCC –> +5V
* GND –>GND
* TXD –> Pin 0 (RX) 
* RXD –> Pin 1 (TX) 
*
* !!!!!!!!!!!!! le module bluetooth  ne doit PAS être apparié à un téléphone ou autre !!!!!!!!!!!!!
*
*/

//**********************************************
// Module bluetooth HC06  envoi de commandes AT
// et affichage de la réponse du module
// D'après http://nicolasz68.blogspot.fr/2012/09/module-bluetooth-jy-mcu-v104-pour.html
//
//
//***********************************************/*  
#include <SoftwareSerial.h>  //Software Serial Port  
#define RxD 2    //Pin 2 pour RX (pin0=serial) vert
#define TxD 3    //Pin 3 pour TX, on peut changer noir
SoftwareSerial BTSerie(RxD,TxD);  

void setup()  
{  
  Serial.begin(57600);    //115200 si on veut
  delay(500);  
  Serial.println("Bonjour - Pret pour les commandes AT");  
  // Configuration du bluetooth  
  pinMode(RxD, INPUT);  
  pinMode(TxD, OUTPUT);  
  BTSerie.begin(57600);  //57600
  delay(500);  
  BTSerie.print("AT+VERSION");  //Demande le N° de version
  delay(1000);  
  // BTSerie.print("\n");  
}  
void loop()  
{  
  char recvChar;  
  //On lit caractere par caractere sur le BTSerie et on affiche sur le Terminal Serie  
  if (BTSerie.available()) {  
    recvChar = BTSerie.read();  
    Serial.print(recvChar);  
  }  
  // Serial.write(blueToothSerial.read());  
  if (Serial.available()) {  
    recvChar = Serial.read();  
    BTSerie.write(recvChar);  
  }  
}  

