/* 
*  
* Une liaison série avec bluetooth
* Une liaison série avec le PC
* Attention, les données ne peuvent pas être envoyées simultanément sur les deux ports
* avant de téléverser le fichier, déconnecter RX.
* 
* * Attention Serial pour la console PC, AlSerial pour la nouvelle bluetooth
* hc06 clignote - la tablette intéragit avec l'arduino ! 
* 
* on peut utiliser The Blueterm free Android app sur android pour communiquer, disponible sur google play
* le code d'appairage par défaut du module bluetooth est 1234
* 
* ATTENTION le signal TX doit être transformé en 3.3 V et pas le 5V d'origine. Il faut donc faire un diviseur de voltage
* 
* VCC to arduino 5V
* GND to arduino GND
* 
*/ 

#include <SoftwareSerial.h>   // nécessaire pour avoir plusieurs flux série (console sur pc et téléphone par exemple)

SoftwareSerial AlSerial(2, 3); // RX HC06 en 3 (would be TX arduino Pin), TX HC06 en 2

// constants  


// pins
int WhiteLed = 7;

void setup() {
 
 
  delay(1000);

  //Connexion série entre PC et arduino 
  Serial.begin(57600);  
  // connexion série entre arduino et module bluetooth
  delay(1000);
  AlSerial.begin(57600);    
  
  //setup pins.   définit le statut des pins comme entrées / sorties
  pinMode(WhiteLed, OUTPUT);

  
  Serial.println("Setup fait.");    // impression sur console PC
  AlSerial.println("Saisir a pour blinker, r pour arreter " );
}

void loop() {
  
  if ( Serial.available() > 0 ) {     	// Wait for user input 
    int state = Serial.read() ;	// lit 1 octet de la console série
    switch ( state ) {
      case 'a' : 
        Serial.print( " 'a' recu : " );   
        Serial.write( state ) ;
        if ( AlSerial.available() > 0 ) {
          AlSerial.println( "je fais blinker" ) ; 
        }
        Serial.println( state ) ;
        digitalWrite(WhiteLed, HIGH);       
        break ;
      case 'r' : 
        Serial.print( " 'r' recu : " );
        Serial.write( state ) ;
        if ( AlSerial.available() > 0 ) {
          AlSerial.println( "stop clignoter" ) ; 
        }
        Serial.println( state ) ;
        digitalWrite(WhiteLed, LOW);
        break ;
       default : 
         Serial.println( "'a' ou 'r' seulement" ) ;
    }
  }
if ( AlSerial.available() > 0 ) {       // Wait for user input 
    int Btate = AlSerial.read() ; // lit 1 octet de la console série
    switch ( Btate ) {
      case 'a' : 
        AlSerial.print( " 'a' recu : " );   
        AlSerial.write( Btate ) ;
        if ( Serial.available() > 0 ) {
          Serial.println( "je fais blinker" ) ; 
        }
        AlSerial.println( Btate ) ;
        digitalWrite(WhiteLed, HIGH);
        break ;
      case 'r' : 
        AlSerial.print( " 'r' recu : " );
        Serial.write( Btate ) ;
        if ( Serial.available() > 0 ) {
          Serial.println( "stop clignoter" ) ; 
        }
        AlSerial.println( Btate ) ;
        digitalWrite(WhiteLed, LOW);
        break ;
       default : 
         AlSerial.println( "'a' ou 'r' seulement" ) ;
    }
  } 
}


