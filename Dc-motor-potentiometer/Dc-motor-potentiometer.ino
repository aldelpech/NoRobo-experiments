/*
 * Faire fonctionner deux moteurs DC avec un potentiomètre
 * Pour identifier la valeur à partir de laquelle le moteur tourne effectivement
 * 
 * Montage des deux moteurs DC autour d'un L293D
 * broche centrale du potentiomèetre sur broche A0 arduino
 *
 * pour mes moteurs DC multiplo, ils avancent à partir de 75
 * ils doivent donc recevoir des commandes entre 75 et 255
 * 
 * Anne-Laure 28/04/2016
 * 
  */

//motor control -------------------------------------------------------

#define MAX_POWER   255   // up to 255

// moteur de gauche
int motorG_enable = 10; //pwm sur timer1
int motorG_forward = 13;
int motorG_backward = 12;

// moteur de droite 
int motorD_enable = 9; //pwm sur timer1
int motorD_forward = 7;
int motorD_backward = 8;

const int potentiometre = A0;
int pot = 0  ; // valeur lue par le potentiomètre

//motor numbers
#define L_Motor 0
#define R_Motor 1

uint16_t MotorPower = 0; // 0 to 255, 255==100% power

bool Robot_Direction = true ; // avant ou arrière 


/****************************************************************************************************
 * SETUP
 ****************************************************************************************************/
 
void setup()
{

  // initialisation potentiomètre
  pinMode(potentiometre, INPUT) ;
  
  //on initialise les pins du moteur 1
  pinMode(motorG_forward, OUTPUT);
  pinMode(motorG_backward, OUTPUT);
  pinMode(motorG_enable, OUTPUT);
 
  //on initialise les pins du moteur 2
  pinMode(motorD_forward, OUTPUT);
  pinMode(motorD_backward, OUTPUT);
  pinMode(motorD_enable, OUTPUT);
  
  // moteurs à l'arrêt
  motorPowerOff();

  Serial.begin(57600);

  Serial.println("Setup termine"); 

  Serial.println("Fais bouger le potentiometre et note la valeur a partir de laquelle les moteurs bougent");       
}

/****************************************************************************************************
 * LOOP
 ****************************************************************************************************/
 
void loop()
{
  pot = analogRead(potentiometre);
  MotorPower = map( pot, 0, 1023, 0, MAX_POWER ) ;  

  //run motors
  MoveMotors(R_Motor, Robot_Direction, MotorPower); 
  MoveMotors(L_Motor, Robot_Direction, MotorPower); 

  Serial.print("Valeur : "); 
  Serial.println(MotorPower);

  
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








