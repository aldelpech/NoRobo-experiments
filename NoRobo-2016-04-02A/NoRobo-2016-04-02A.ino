/****************************************************************
* contrôler la vitesse et le sens de rotation de moteurs DC
* ce programme fait tourner le robot dans un sens puis l'autre
* en commencant par sens inverse des aiguilles d'une montre
*
* sources 
* http://www.zem.fr/arduino-controler-des-moteurs-dc-avec-le-composant-l293d/#codesyntax_1
* 
****************************************************************/

// moteur de gauche
int motorG_enable = 11; //pwm
int motorG_forward = 13;
int motorG_backward = 12;

// moteur de droite 
int motorD_enable = 10; //pwm
int motorD_forward = 8;
int motorD_backward = 9;
 
void setup()
{
  //on initialise les pins du moteur 1
  pinMode(motorG_forward, OUTPUT);
  pinMode(motorG_backward, OUTPUT);
  pinMode(motorG_enable, OUTPUT);
 
  //on initialise les pins du moteur 2
  pinMode(motorD_forward, OUTPUT);
  pinMode(motorD_backward, OUTPUT);
  pinMode(motorD_enable, OUTPUT);
 
}
 
void loop()
{
 
  SetMotorD(175, true);   // 0 to 255, true marche avant, false marche arrière
  SetMotorG(255, false);   // 0 to 255, true marche avant, false marche arrière

  delay(2500) ;
 
  StopMotors(200) ;

  // recommence dans l'autre direction
  SetMotorD(255, false);  // 0 to 255, true marche avant, false marche arrière
  SetMotorG(175, true);  // 0 to 255, true marche avant, false marche arrière  

  delay(2500) ;
 
  StopMotors(200) ;
}
 
//Fonction qui set le moteur de gauche
void SetMotorG(int speed, boolean reverse)
{
  analogWrite(motorG_enable, speed);
  digitalWrite(motorG_forward, ! reverse);
  digitalWrite(motorG_backward, reverse);
}
 
//Fonction qui set le moteur de droite
void SetMotorD(int speed, boolean reverse)
{
  analogWrite(motorD_enable, speed);
  digitalWrite(motorD_forward, ! reverse);
  digitalWrite(motorD_backward, reverse);
}

//Fonction qui arrête les 2 moteurs pendant une certaine durée
void StopMotors(int duree)
{
  // stoppe les moteurs
  digitalWrite(motorG_enable, LOW);
  digitalWrite(motorD_enable, LOW); 
  delay(duree) ;
}
