/*
 * contient le code de NoRobo-Gyro-2016-04-28-B.ino mais ne va pas mesurer l'angle 
 * PID out est simulé par un potentiomètre qui peut prendre -100 à 100
 * 
 * fonctionne correctement quand 
 *    MotorPower = 0
 *    
 * 
 * but = déterminer la formule de calcul des moteurs pour que
 * si motorpower = 0 (le robot est à l'arrêt) :
 *      PID négatif (angle vers l'arrière) : le robot recule
 *      PID positif (angle vers l'avant) : le robot avance  
 * si motorpower > 0 (il va vers l'avant) :
 *      PID négatif (angle vers l'arrière) : le robot ralentit
 *      PID positif (angle vers l'avant) : le robot accèlère 
 * si motorpower < 0 (il va vers l'arrière) : 
 *      PID négatif (angle vers l'arrière) : le robot accélère
 *      PID positif (angle vers l'avant) : le robot ralentit 
 * 
 */


#pragma GCC optimize ("3")

#include <Wire.h>

//main loop--------------------------------------------------------

bool power = true;            //run only when power is true

uint16_t freqCounter = 0;
uint16_t oldfreqCounter = 0;
uint16_t loop_time = 0;         //how fast is the main loop running


//Bluetooth input stuff------------------------------------------
#define    STX          0x02	// debut d'ordre
#define    ETX          0x03	// fin d'ordre
#define    SLOW         750                             // Datafields refresh rate (ms)
#define    FAST         250                             // Datafields refresh rate (ms)

int16_t joyY, joyX;                                     //smartphone joystick input values

uint8_t cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};      // bytes received
uint8_t buttonStatus = 0;                       // first Byte sent to Android device
long previousMillis = 0;                        // will store last time Buttons status was updated
long sendInterval = SLOW;                       // interval between Buttons status transmission (milliseconds)
//motor control-------------------------------------------------------

#define MAX_POWER   255   // up to 255

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

int16_t MotorPower = -90; // -255 to 255, 255==100% power !!! int8 et pas uint16 !!!

uint8_t idl_motor_power = 73; // measured with Dc-motor-potentiometer.ino sketch and a potentiometer

uint8_t resp_rate = 1; //bluetooth remote joystick multiplier

bool direction = false ; // sens de marche du robot

//rotation speed for turning
int8_t rot_Speed = 0;

float robot_speed;

// speed of motors, 0 TO 255
int16_t R_Speed = 0;    // doit être int16 pour aller jusqu'à 256 et pas 128
int16_t L_Speed = 0;

#define PID_out_max 100
#define PID_out_min -100


// potentiometer stuff
const int potentiometre = A0;
int pot = 0  ; // valeur lue par le potentiomètre
int8_t PID_out = 0  ; // valeur modifiée du potentiomètre

// pour voir ce qui se passe
int8_t MotorP_0 ;
int8_t MotorP_1 ;


/****************************************************************************************************
 * SETUP
 ****************************************************************************************************/
 
void setup()
{

  // initialisation potentiomètre
  pinMode(potentiometre, INPUT) ;
 
  Serial.begin(57600);
  delay(500) ;

  //Setup MOTOR Controller
  Bl_Setup();

  Serial.println("Setup termine"); 

  Serial.print("POT");       // prints a label
  Serial.print("\t");              // prints a tab

  Serial.print("PID");  
  Serial.print("\t");      

  Serial.print("P_0"); 
  Serial.print("\t");   

  Serial.print("P_1");
  Serial.print("\t");

  Serial.print("R_Speed");
  Serial.print("\t");    

  Serial.print("L_Speed");
  Serial.print("\t");  

  Serial.print("DIR");
  Serial.print("\t"); 

  Serial.print("\r\n");
}

/****************************************************************************************************
 * LOOP
 ****************************************************************************************************/
 
void loop()
{

  //run if on sent from smartphone and battery is charged
  if ( power == true ) PID_and_motor_comd();
  else
  {
    motorPowerOff();               //turn motors off if battery is discharged / powered off
  }

  Serial.print(pot);       // prints a label
  Serial.print("\t");              // prints a tab

  Serial.print(PID_out);  
  Serial.print("\t");      

  Serial.print(MotorP_0); 
  Serial.print("\t");   

  Serial.print(MotorP_1);
  Serial.print("\t");

  Serial.print(R_Speed);
  Serial.print("\t");    

  Serial.print(L_Speed);
  Serial.print("\t");  

  Serial.print(direction);
  Serial.print("\t"); 

  Serial.print("\r\n");

}


//--------------------------------------------------------------------------------------
//angle PID calculation and motor command signal generation-----------------------------
//--------------------------------------------------------------------------------------
void PID_and_motor_comd()
{

  pot = analogRead(potentiometre);
    
  PID_out = map (pot, 0, 1023, -120, 120 ) ;

  if (PID_out > PID_out_max) PID_out = PID_out_max;
  else if (PID_out < PID_out_min) PID_out = PID_out_min;
  
  // stocker valeur MotorPower avant
  MotorP_0 =  MotorPower ;

  int speed ;
  if ( (MotorPower <= -idl_motor_power) || (MotorPower >= idl_motor_power) ) {

    Serial.print( "MotorP > idle ---") ;

    // MotorPwr dans zone qui actionne les moteurs

    if ( ( MotorPower * PID_out ) < 0 ) {

      Serial.print(" MotorP x PID <0 **** ");
      // signes de MotorPower et PID_out opposés

      if ( abs(MotorPower) < abs(PID_out) ) {

        // éliminer la zone -idl -- idl
        if (PID_out < 0) {
         speed = MotorPower + PID_out - idl_motor_power  ;
        } else {
         speed = MotorPower + PID_out + idl_motor_power  ;          
        }

        Serial.println(" MotorP<PID ---> correction ");   
                
      } else {
      
        speed = MotorPower + PID_out ;
        Serial.println(" MotorP>PID RAS ");     
           
      }
      
    } else {

      speed = MotorPower + PID_out ;
      Serial.println(" MotorP x PID >=0  ---- RAS ");

    }
     
  } else { 

    // MotorPower est dans la zone "idle", équivalent à 0
    if ( PID_out <= 0) {
        speed = -idl_motor_power + PID_out ;    // valeur négative
        Serial.println(" MotorPwr  ds zone idl et PID < 0--------------- ");
    } else {
        speed = +idl_motor_power + PID_out ;    // valeur positive    
        Serial.println(" MotorPwr  ds zone idl et PID > 0--------------- ");
    }
    
  }

  // stocker valeur pour affichage
  MotorP_1 =  speed ;  

  if ( speed < - idl_motor_power ) {
    // si speed < 0 le robot va vers l'AR
    direction = false ; 
    speed = abs( speed ) ;  // speed redevient positif
      
  } else if ( speed > idl_motor_power ) {
      
    // si speed > 0 le robot va vers l'AV
    direction = true ;
      
  } else if ( ( speed >= - idl_motor_power ) && ( speed < 0 )  ) {
    
    speed = 2*idl_motor_power + speed ; 
    direction = true ;    
    
  } else if ( ( speed <= idl_motor_power ) && ( speed >= 0 )  ) {

    speed = - (speed - 2*idl_motor_power)  ;
    direction = false ;
    
  } else {
    Serial.println("!!!! Cas non prevu ----------------------- ");
    motorPowerOff() ;
  }

  // speed ne doit pas excéder 255
  if ( speed > 255 ) {
    speed = 255 ;
  }
  
  R_Speed = speed;
  L_Speed = speed;

  if ( ( R_Speed < 256 ) && ( L_Speed < 256 ) && ( R_Speed > 0 ) && ( L_Speed > 0  ) ) {
    //run motors 
    // MoveMotors(R_Motor, direction, R_Speed);
    // MoveMotors(L_Motor, direction, L_Speed);         
  } else {
    // do not move the motors
  }

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

  // switch off PWM Power
  motorPowerOff();
}


//Fonction qui arrête les 2 moteurs 
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





