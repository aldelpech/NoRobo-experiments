/* This will work for a HC-05 
* 
* 
* cablage 
* VCC –> +5V
* GND –>GND
* TXD –> Pin 0 (RX) 
* RXD –> Pin 1 (TX) 
* Si HC-05 : EN (ou Key) relié au 3.3V de l'arduino
* 
* Le HC-05 doit clignoter lentement (2 sec) s'il est en mode commandes AT
* La console série de l'ordi doit être sur 57600 bauds et "les deux, NL et CR" pour les fins de ligne
* le module bluetooth  ne doit PAS être apparié à un téléphone ou autre
* 
* will not work with a HC-06 which doesn't want \r\n for an end of line
*/

/**********************************************
* Module bluetooth HC05  envoi de commandes AT
* et affichage de la réponse du module
* Source 
* http://www.instructables.com/id/AT-command-mode-of-HC-05-Bluetooth-module/?ALLSTEPS
* http://www.instructables.com/id/Modify-The-HC-05-Bluetooth-Module-Defaults-Using-A/?ALLSTEPS
*
************************************************/
  
#include <SoftwareSerial.h>  //Software Serial Port  
#define RxD 2    //Pin 2 pour arduino RX (pin0=serial) 
#define TxD 3    //Pin 3 pour arduino TX 
SoftwareSerial BTSerie(RxD,TxD);  

void setup()  
{  
  Serial.begin(57600);    //115200 si on veut
  delay(500);  
  Serial.println("Bonjour - Pret pour les commandes AT");  
  Serial.println("Le HC-05 doit clignoter lentement (2 secondes)");
  // Configuration du bluetooth  
  pinMode(RxD, INPUT);  
  pinMode(TxD, OUTPUT);  
  BTSerie.begin(38400);  //57600 / 38400
  delay(500);  
  BTSerie.print("AT+VERSION?");  //Demande le N° de version
  BTSerie.print("\r\n");     // sur HC-05, toutes les commandes doivent se terminer par \r\n

  // afficher ce que le module bluetooth répond
    Serial.print( BTSerie.read() );     // afficher sur console ce qui est lu sur BT
  // si tout va bien, c'est le n° de version puis OK qui s'affiche
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

