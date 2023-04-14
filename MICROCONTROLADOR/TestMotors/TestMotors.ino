#include "SimplyAtomic.h"
#include "BTS7960.h"
#include "Encoder.h"

//Tiempos de prueba
int t1 = 5000;
int t2 = 6000;
int t3 = 7500;
int t4 = 8000;

// MOTOR 0 -----------------------------------------------------------

//Control con ZK-5AD
#define RPWM0 2  // Azul
#define LPWM0 3  // Morado
//Encoder
Encoder myEnc0(32, 34); 
long position0 = -999; 
//Resolucion
float resolution0 = 360.0 / (34.6 * 34.02);

// MOTOR 1 -----------------------------------------------------------

// Control con BTS7960
#define RPWM1 12 // Verde
#define LPWM1 13 // Azul
#define EN1 11
//Encoder
//ENCODER_A1 26  // Amarillo
//ENCODER_B1 28  // Blanco
Encoder myEnc1(26, 28);
long position1 = -999; 
//Resolucion
float resolution1 = 360.0 / 8400.0;
//Creacion de motor
BTS7960 motorController(EN1, LPWM1, RPWM1);

// MOTOR 2 -----------------------------------------------------------

//Control con ZK-5AD
#define RPWM2 8  // Azul
#define LPWM2 9  // Morado
//Encoder
//ENCODER_A2 24  // Amarillo
//ENCODER_B2 22  // Verde
Encoder myEnc2(22, 24); 
long position2 = -999;  
//Resolucion
float resolution2 = 360.0 / (44.0 * 34.02);


void setup() {
  Serial.begin(115200);

  // Configuracion Motor 0
  pinMode(LPWM0, OUTPUT);
  pinMode(RPWM0, OUTPUT);
  analogWrite(LPWM0, 0);
  analogWrite(RPWM0, 0);

  // Configuracion Motor 1
  

  // Configuracion Motor 2
  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  analogWrite(LPWM2, 0);
  analogWrite(RPWM2, 0);

  delay(2000);
}

void loop() {
  unsigned long currentMillis = millis();
  
  float newP0, newP1, newP2;
  newP0 = myEnc0.read()*resolution0;
  newP1 = myEnc1.read()*resolution1;
  newP2 = myEnc2.read()*resolution0; 
  
  if ((newP0 != position0) || (newP1 != position1) || (newP2 != position2)) {
    Serial.print(millis()/1000.0);
    Serial.print(";");
    //Serial.println(newP0);
    //Serial.print(", P1 = ");
    Serial.println(newP1);
    //Serial.print(", P2 = ");
    //Serial.println(newP2);
    //position0 = newP0;
    //position1 = newP1;
    //position2 = newP2;
  } 
   
  int vel = 100;
  
  if (currentMillis >= t1 && currentMillis < t2) {
    setMotor(1, vel, false);
  } else if (currentMillis >= t2) {
    setMotor(1, 0, false);
  }
}


void setMotor(int motor, int vel, bool dir) {
  switch(motor){
    case 0:
      if (dir) {
        analogWrite(LPWM0, LOW);
        analogWrite(RPWM0, vel);
        
        //delayMicroseconds(100);
        
      } else {
        analogWrite(RPWM0, LOW);
        analogWrite(LPWM0, vel);
        
        //delayMicroseconds(100);
        
      }
      break;    
    case 1:
      if (!motorController.isEnabled()){
        motorController.Enable();        
      }
      if (dir) {
        motorController.TurnRight(vel);
      } else {
        motorController.TurnLeft(vel);
      }        
      break;
    case 2:
      if (dir) {
        analogWrite(RPWM2, LOW);
        analogWrite(LPWM2, vel);
      } else {
        analogWrite(LPWM2, LOW);
        analogWrite(RPWM2, vel);
      }
      break;
    default:
      analogWrite(LPWM0, 0);
      analogWrite(RPWM0, 0);
      motorController.Disable();
      analogWrite(LPWM2, 0);
      analogWrite(RPWM2, 0);           
  }
    

}
