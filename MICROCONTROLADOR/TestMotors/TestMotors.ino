#include "SimplyAtomic.h"
#include "BTS7960.h"
#include "Encoder.h"

//Tiempos
long previousMillis = 0;

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
float position0 = -999;
//Resolucion
float resolution0 = 360.0 / (34.6 * 34.02);

// MOTOR 1 -----------------------------------------------------------

// Control con BTS7960
#define RPWM1 12 // Verde
#define LPWM1 13 // Azul
#define EN1 11
//Encoder
Encoder myEnc1(26, 28);
float position1 = -999;
//Resolucion
float resolution1 = 360.0 / 8400.0;
//Creacion de motor
BTS7960 motorController(EN1, LPWM1, RPWM1);

// MOTOR 2 -----------------------------------------------------------

//Control con ZK-5AD
#define RPWM2 8  // Azul
#define LPWM2 9  // Morado
//Encoder
Encoder myEnc2(22, 24);
float position2 = -999;
//Resolucion
float resolution2 = 360.0 / (34.6 * 34.02);

//Referencias
float Ref0 = 0;
float Ref1 = 0;
float Ref2 = 10;

//Controladores
float Tf = 0.105;
float Ts = 0.001;
int TsC = 1; //1 ms de tiempo de refresco para los controladores

float kp0 = 8; //0.0698;
float kd0 = 0.1; //0.00546;
float kp1 = 6;//0.254;
float kd1 = 0.05;//0.0102;
float kp2 = 10;
float kd2 = 0.1;//0.00566;


float E0, E0p, E1, E1p, E2, E2p = 0;
float cmdPD0, cmdPD1, cmdPD2 = 0;

boolean sentido0 = true;
boolean sentido1 = true;
boolean sentido2 = true;

int pwmMax = 255;


//Ángulos
long newP0, newP1, newP2;

int pwmDuty0, pwmDuty1, pwmDuty2 = 0;


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

  controlPDplus();
  Serial.print("Angle: ");
  Serial.print(position2);
  Serial.print("PWM: ");
  Serial.println(cmdPD2);  

  /*
  int vel = 200;

  if (currentMillis >= t1 && currentMillis < t2) {
    setMotor(1, vel, false);
  } else if (currentMillis >= t2 && currentMillis < t3) {
    setMotor(1, 0, false);
  } else if (currentMillis >= t3 && currentMillis < t4) {
    setMotor(1, vel, true);
  } else if (currentMillis >= t4) {
    setMotor(1, 0, true);
  }*/

  /*

    motorController.Enable();
    delay(1000);

    delay(1000);

    delay(1000);
  */

}


void controlPDplus() {
  unsigned long currentMillis = millis();

  //Transcurridos 5 segundos comienza a funcionar el controlador

  if (currentMillis >= 5000) {

    /*
      ATOMIC() {
      posicion = (float(-theta * 360.0 / resolution)) + 180;
      posicion2 = (float(theta2 * 360.0 / resolution)) - posicion + 15;
      }*/

    //Se realiza la lectura de los encoder
    newP0 = myEnc0.read();
    newP1 = myEnc1.read();
    newP2 = myEnc2.read();

    //Se actualiza el valor angular en grados de cada articulación
    position0 = newP0 * resolution0;
    position1 = newP1 * resolution1;
    position2 = newP2 * resolution2;

    //Se actualiza el valor del error anterior de cada eslabón
    E0p = E0;
    E1p = E1;
    E2p = E2;

    //Se actualiza el error en cada eslabón
    E0 = float(Ref0 - position0);
    E1 = float(Ref1 - position1);
    E2 = float(Ref2 - position2);

    //Si el error angular es inferior a 1°, se hace que el error sea nulo
    if (abs(E0) <= 1) {
      E0 = 0;
    }

    if (abs(E1) <= 1) {
      E1 = 0;
    }

    if (abs(E2) <= 0.1) {
      E2 = 0;
    }

    //Se calcula la acción de control
    cmdPD0 = (kp0 - kd0 * (1 / (Tf + Ts))) * (E0) + kd0 * (1 / (Tf + Ts)) * E0p;
    cmdPD1 = (kp1 - kd1 * (1 / (Tf + Ts))) * (E1) + kd1 * (1 / (Tf + Ts)) * E1p;
    cmdPD2 = (kp2 - kd2 * (1 / (Tf + Ts))) * (E2) + kd2 * (1 / (Tf + Ts)) * E2p;
  }

  
  if (currentMillis - previousMillis >= TsC) {
    //Se actualiza el timer
    previousMillis = currentMillis;

    //Se suman las dos señales de control
    float Cmd0 = cmdPD0; // + 260.0286 * ((g * m2 * l1 * cos((posicion) * (PI / 180)) + g * m2 * lc2 * cos((posicion2 + posicion) * (PI / 180)) + g * lc1 * m1 * cos((posicion) * (PI / 180))));  //+ g*m2*l1*cos(posicion)+g*m2*lc2*cos(posicion+posicion2)+g*lc1*m1*cos(posicion);
    float Cmd1 = cmdPD1;
    float Cmd2 = cmdPD2; // + (g * lc2 * m2 * cos((posicion + posicion2) * (PI / 180))) * 260.0286;

    //Se verifica el sentido de cada motor
    if (Cmd0 >= 0) {
      sentido0 = false;
    } else {
      sentido0 = true;
    }

    if (Cmd1 >= 0) {
      sentido1 = false;
    } else {
      sentido1 = true;
    }

    if (Cmd2 >= 0) {
      sentido2 = true;
    } else {
      sentido2 = false;
    }

    //Se obtiene el valor de cmd positivo
    Cmd0 = abs(Cmd0);
    Cmd1 = abs(Cmd1);
    Cmd2 = abs(Cmd2);

    //Función de saturación
    float CmdLim0 = min(max(Cmd0, 0), pwmMax);    // Saturated Control Output
    float CmdLim1 = min(max(Cmd1, 0), pwmMax);    // Saturated Control Output
    float CmdLim2 = min(max(Cmd2, 0), pwmMax);    // Saturated Control Output

    //Ciclos útil de PWM
    pwmDuty0 = int(CmdLim0);
    pwmDuty1 = int(CmdLim1);
    pwmDuty2 = int(CmdLim2);

    //Solo ejecuta cuando hayan pasado 5 segundos
    if (currentMillis >= 5000) {
      setMotor(0, pwmDuty0, sentido0);
      setMotor(1, pwmDuty1, sentido1);
      setMotor(2, pwmDuty2, sentido2);
    }
  }
}



void setMotor(int motor, int vel, bool dir) {
  switch (motor) {
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
      motorController.Enable();
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
