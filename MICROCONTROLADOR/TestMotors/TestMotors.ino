#include "SimplyAtomic.h"
#include "BTS7960.h"
#include "Encoder.h"
#include <Servo.h>

//Tiempos
long previousMillis, previousMillisRef = 0;

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
float p0On = 0;
//Resolucion
float resolution0 = 360.0 / (34.6 * 34.02);

// MOTOR 1 -----------------------------------------------------------

// Control con BTS7960
#define RPWM1 12  // Verde
#define LPWM1 13  // Azul
#define EN1 11
//Encoder
Encoder myEnc1(26, 28);
float position1 = -999;
float p1On = 101.10;
//Resolucion
float resolution1 = 360.0 / 8400.0;
//Creacion de motor
BTS7960 motorController(EN1, LPWM1, RPWM1);

// MOTOR 2 -----------------------------------------------------------

//Control con ZK-5AD
#define RPWM2 8  // Azul
#define LPWM2 9  // Morado
//Encoder
Encoder myEnc2(24, 22);
float position2 = -999;
float p2On = -165;
//Resolucion
float resolution2 = 360.0 / (34 * 34.02);

//Servo
Servo myservo;  // crea el objeto servo

//Referencias
float Ref0 = 0;
float Ref1 = 110;
float Ref2 = -165;
int Ref3 = 90;

/*
float Ref0Var[] = { 0, -0.1849, -0.1849, -0.1623, -0.1453, -0.1355, -0.1318, -0.1302, -0.1252, -0.1120, -0.0894, -0.0599, -0.0292, -0.0042, 0.0083, 0.0041, -0.0171, -0.0506, -0.0879, -0.1204, -0.1436, -0.1583, -0.1692, -0.1831, -0.2060, -0.2402, -0.2816, -0.3208, -0.3482, -0.3600, -0.3593, -0.3536, -0.3510, -0.3576, -0.3759, -0.4032, -0.4321, -0.4534, -0.4605, -0.4512, -0.4280, -0.3963, -0.3623, -0.3316, -0.3080, -0.2918, -0.2803, -0.2690, -0.2541, -0.2341, -0.2101, -0.1849, 0, -0.1777, -0.1777, -0.1286, -0.0728, -0.0097, 0.0617, 0.0617, -0.0112, -0.0980, -0.2021, -0.3272, -0.3272, -0.3871, -0.4344, -0.4724, -0.5034, -0.5034, -0.4102, -0.3250, -0.2476, -0.1777, -0.1777, 0 };
float Ref1Var[] = { 1.7645, 1.3575, 1.1375, 1.1340, 1.1242, 1.1100, 1.0945, 1.0809, 1.0718, 1.0675, 1.0660, 1.0644, 1.0597, 1.0501, 1.0349, 1.0156, 0.9951, 0.9775, 0.9654, 0.9577, 0.9505, 0.9388, 0.9190, 0.8903, 0.8557, 0.8230, 0.8026, 0.8026, 0.8230, 0.8557, 0.8903, 0.9190, 0.9388, 0.9505, 0.9577, 0.9654, 0.9775, 0.9951, 1.0156, 1.0349, 1.0501, 1.0597, 1.0644, 1.0660, 1.0675, 1.0718, 1.0809, 1.0945, 1.1100, 1.1242, 1.1340, 1.3575, 0, 1.3680, 1.1571, 1.1287, 1.0987, 1.0676, 1.0366, 1.0366, 0.9762, 0.9050, 0.8218, 0.7271, 0.7271, 0.8218, 0.9050, 0.9762, 1.0366, 1.0366, 1.0676, 1.0987, 1.1287, 1.1571, 1.3680, 1.7645 };
float Ref2Var[] = { -2.8798, -1.5708, -1.5708, -1.5708, -1.5708, -1.5708, -1.5709, -1.5709, -1.5709, -1.5709, -1.5709, -1.5709, -1.5709, -1.5709, -1.5710, -1.5710, -1.5710, -1.5710, -1.5710, -1.5710, -1.5710, -1.5711, -1.5711, -1.5711, -1.5711, -1.5711, -1.5711, -1.5711, -1.5711, -1.5711, -1.5711, -1.5711, -1.5711, -1.5710, -1.5710, -1.5710, -1.5710, -1.5710, -1.5710, -1.5710, -1.5709, -1.5709, -1.5709, -1.5709, -1.5709, -1.5709, -1.5709, -1.5709, -1.5708, -1.5708, -1.5708, -1.5708, 0, -1.5708, -1.5707, -1.5708, -1.5709, -1.5709, -1.5710, -1.5710, -1.5710, -1.5711, -1.5711, -1.5712, -1.5712, -1.5711, -1.5711, -1.5710, -1.5710, -1.5710, -1.5709, -1.5709, -1.5708, -1.5707, -1.5708, -2.8798};
*/

float Ref0Var[] = { 0, 0.1122, 0.2244, 0.3366, 0.4488, 0.5610, 0.6732, 0.7854, 0.8976, 1.0098, 1.1220, 1.2342, 1.3464, 1.4586, 1.5708 };
float Ref1Var[] = { 1.7645, 1.6385, 1.5125, 1.3864, 1.2604, 1.1343, 1.0083, 0.8823, 0.7562, 0.6302, 0.5042, 0.3781, 0.2521, 0.1260, 0 };
float Ref2Var[] = { -2.8798, -2.6741, -2.4684, -2.2627, -2.0570, -1.8513, -1.6456, -1.4399, -1.2342, -1.0285, -0.8228, -0.6171, -0.4114, -0.2057, 0 };
float Ref3Var[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/*
float vq1[] = { 0, -0.9245, 0, 0.1132, 0.0848, 0.0489, 0.0185, 0.0080, 0.0253, 0.0658, 0.1131, 0.1475, 0.1536, 0.1247, 0.0627, -0.0210, -0.1061, -0.1675, -0.1863, -0.1627, -0.1162, -0.0733, -0.0546, -0.0696, -0.1145, -0.1707, -0.2071, -0.1961, -0.1371, -0.0589, 0.0033, 0.0285, 0.0132, -0.0331, -0.0915, -0.1364, -0.1444, -0.1068, -0.0353, 0.0465, 0.1158, 0.1587, 0.1701, 0.1533, 0.1183, 0.0811, 0.0575, 0.0563, 0.0744, 0.1001, 0.1203, 0.1258, 0.9245, -0.8886, 0, 0.2458, 0.2787, 0.3158, 0.3569, 0, -0.3645, -0.4341, -0.5204, -0.6258, 0, -0.2994, -0.2365, -0.1899, -0.1550, 0, 0.4658, 0.4262, 0.3869, 0.3495, 0, 0.8886 };
float vq2[] = { 0, -2.0350, -1.1002, -0.0173, -0.0489, -0.0710, -0.0780, -0.0678, -0.0454, -0.0216, -0.0074, -0.0082, -0.0234, -0.0483, -0.0757, -0.0968, -0.1022, -0.0879, -0.0608, -0.0383, -0.0360, -0.0586, -0.0991, -0.1436, -0.1726, -0.1636, -0.1022, 0, 0.1022, 0.1636, 0.1726, 0.1436, 0.0991, 0.0586, 0.0360, 0.0383, 0.0608, 0.0879, 0.1022, 0.0968, 0.0757, 0.0483, 0.0234, 0.0082, 0.0074, 0.0216, 0.0454, 0.0678, 0.0780, 0.0710, 0.0489, 1.1175, -6.7876, 6.8400, -1.0545, -0.1419, -0.1502, -0.1553, -0.1550, 0, -0.3019, -0.3562, -0.4158, -0.4737, 0, 0.4737, 0.4158, 0.3562, 0.3019, 0, 0.1550, 0.1553, 0.1502, 0.1419, 1.0545, 1.9826 };
float vq3[] = { 0, 6.5447, 0.0004, -0.0000, -0.0001, -0.0002, -0.0002, -0.0001, -0.0001, -0.0000, -0.0000, -0.0000, -0.0000, -0.0001, -0.0001, -0.0001, -0.0001, -0.0001, -0.0001, -0.0000, -0.0000, -0.0000, -0.0001, -0.0001, -0.0001, -0.0001, -0.0000, 0, 0.0000, 0.0001, 0.0001, 0.0001, 0.0001, 0.0000, 0.0000, 0.0000, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0000, 0.0000, 0.0000, 0.0000, 0.0001, 0.0001, 0.0002, 0.0002, 0.0001, -0.0003, 7.8542, -7.8540, 0.0004, -0.0004, -0.0003, -0.0003, -0.0002, 0, -0.0003, -0.0003, -0.0002, -0.0002, 0, 0.0002, 0.0002, 0.0003, 0.0003, 0, 0.0002, 0.0003, 0.0003, 0.0004, -0.0004, -6.5450};
*/

float vq1[] = { 0, 0.1047, 0.1047, 0.1047, 0.1047, 0.1047, 0.1047, 0.1047, 0.1047, 0.1047, 0.1047, 0.1047, 0.1047, 0.1047, 0.1047 };
float vq2[] = { 0, -0.1176, -0.1176, -0.1176, -0.1176, -0.1176, -0.1176, -0.1176, -0.1176, -0.1176, -0.1176, -0.1176, -0.1176, -0.1176, -0.1176 };
float vq3[] = { 0, 0.1920, 0.1920, 0.1920, 0.1920, 0.1920, 0.1920, 0.1920, 0.1920, 0.1920, 0.1920, 0.1920, 0.1920, 0.1920, 0.1920 };

/*
float aq1[] = { 0, -4.6225, 4.6225, 0.5659, -0.1419, -0.1797, -0.1518, -0.0525, 0.0867, 0.2024, 0.2366, 0.1717, 0.0307, -0.1447, -0.3097, -0.4185, -0.4257, -0.3067, -0.0940, 0.1180, 0.2324, 0.2145, 0.0934, -0.0748, -0.2246, -0.2810, -0.1818, 0.0547, 0.2950, 0.3910, 0.3110, 0.1260, -0.0765, -0.2317, -0.2917, -0.2245, -0.0402, 0.1880, 0.3578, 0.4086, 0.3466, 0.2143, 0.0574, -0.0844, -0.1747, -0.1863, -0.1179, -0.0061, 0.0905, 0.1286, 0.1010, 0.0275, 3.9934, -9.0652, 4.4428, 1.2289, 0.1644, 0.1857, 0.2054, -1.7844, -1.8223, -0.3479, -0.4316, -0.5270, 3.1288, -1.4971, 0.3145, 0.2329, 0.1746, 0.7751, 2.3291, -0.1982, -0.1964, -0.1868, -1.7477, 4.4428 };
float aq2[] = { 0, -10.1752, 4.6744, 5.4142, -0.1577, -0.1105, -0.0349, 0.0508, 0.1121, 0.1190, 0.0711, -0.0042, -0.0761, -0.1244, -0.1372, -0.1052, -0.0273, 0.0717, 0.1352, 0.1128, 0.0113, -0.1128, -0.2027, -0.2224, -0.1450, 0.0448, 0.3074, 0.5108, 0.5108, 0.3074, 0.0448, -0.1450, -0.2224, -0.2027, -0.1128, 0.0113, 0.1128, 0.1352, 0.0717, -0.0273, -0.1052, -0.1372, -0.1244, -0.0761, -0.0042, 0.0711, 0.1190, 0.1121, 0.0508, -0.0349, -0.1105, 5.3431, -39.5255, 68.1380, -39.4727, 4.5633, -0.0417, -0.0252, 0.0013, 0.7750, -1.5097, -0.2713, -0.2980, -0.2898, 2.3687, 2.3687, -0.2898, -0.2980, -0.2713, -1.5097, 0.7750, 0.0013, -0.0252, -0.0417, 4.5633, 4.6405 };
float aq3[] = { 0, 32.7236  - 32.7217, -0.0022, -0.0004, -0.0002, 0.0000, 0.0002, 0.0002, 0.0002, 0.0001, -0.0000, -0.0001, -0.0002, -0.0002, -0.0001, 0.0000, 0.0001, 0.0002, 0.0001, 0.0000, -0.0001, -0.0001, -0.0001, -0.0000, 0.0001, 0.0002, 0.0002, 0.0002, 0.0002, 0.0001, -0.0000, -0.0001, -0.0001, -0.0001, 0.0000, 0.0001, 0.0002, 0.0001, 0.0000, -0.0001, -0.0002, -0.0002, -0.0001, -0.0000, 0.0001, 0.0002, 0.0002, 0.0002, 0.0000, -0.0002, -0.0023, 39.2730  - 78.5411, 39.2719, -0.0038, 0.0002, 0.0002, 0.0002, 0.0011, -0.0017, 0.0003, 0.0003, 0.0002, 0.0009, 0.0009, 0.0002, 0.0003, 0.0003, -0.0017, 0.0011, 0.0002, 0.0002, 0.0002, -0.0038, -32.7230};
*/

float aq1[] = { 0, 0.0977, 0, 0, 0, 0.0000, -0.0000, 0.0000, 0, 0, 0.0000, -0.0000, 0.0000, 0, 0 };
float aq2[] = { 0, -0.1098, 0.0000, -0.0000, 0, 0, 0.0000, -0.0000, 0.0000, -0.0000, 0.0000, -0.0000, 0.0000, -0.0000, 0 };
float aq3[] = { 0, 0.1792, 0.0000, 0, -0.0000, 0.0000, -0.0000, 0.0000, -0.0000, 0, 0.0000, -0.0000, 0.0000, 0.0000, -0.0000 };


int k = 0;
int pasoTime = 1000;  //200 millis

//Lectura Serial
char oneT = '1';
char zeroT = '0';
float motionProfiles[10] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};


//Controladores
float Tf = 0.105;
float Ts = 0.001;
int TsC = 1;  //1 ms de tiempo de refresco para los controladores

float kp0 = 5;     //0.0698;
float kd0 = 0.1;   //0.00546;
float kp1 = 4;     //0.254;
float kd1 = 0.05;  //0.0102;
float kp2 = 8.3;
float kd2 = 0.12;  //0.00566;


float E0, E0p, E1, E1p, E2, E2p = 0;
float cmdPD0, cmdPD1, cmdPD2 = 0;

boolean sentido0 = true;
boolean sentido1 = true;
boolean sentido2 = true;

int pwmMax = 255;


//Ángulos
long newP0, newP1, newP2;

int pwmDuty0, pwmDuty1, pwmDuty2 = 0;

//Torques
float tau1, tau2, tau3 = 0;


void setup() {
  //Serial.begin(115200);
  Serial.begin(9600);
  Serial2.begin(9600);

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

  //Servo
  myservo.attach(5);  // vincula el servo al pin digital 5
  myservo.write(0);

  delay(2000);
}

void loop() {
  //setMotor(1, 70, false);
  controlPDplus();
}


void controlPDplus() {
  unsigned long currentMillisRef = millis();
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
    position0 = newP0 * resolution0 + p0On;
    position1 = newP1 * resolution1 + p1On;
    position2 = newP2 * resolution2 + p2On;

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

    if (abs(E2) <= 1) {
      E2 = 0;
    }

    //Se calcula la acción de control
    cmdPD0 = (kp0 - kd0 * (1 / (Tf + Ts))) * (E0) + kd0 * (1 / (Tf + Ts)) * E0p;
    cmdPD1 = (kp1 - kd1 * (1 / (Tf + Ts))) * (E1) + kd1 * (1 / (Tf + Ts)) * E1p;
    cmdPD2 = (kp2 - kd2 * (1 / (Tf + Ts))) * (E2) + kd2 * (1 / (Tf + Ts)) * E2p;

    if (currentMillis - previousMillis >= TsC) {
      //Se actualiza el timer
      previousMillis = currentMillis;

      if (currentMillisRef - previousMillisRef >= pasoTime) {
        k += 1;
        if (k >= sizeof(Ref0Var) / sizeof(int)) {
          k = sizeof(Ref0Var) / sizeof(int);
        }
        previousMillisRef = currentMillisRef;
      }

      readSerial();
      Ref0 = motionProfiles[0]+p0On;//Ref0Var[k] * (180 / PI);
      Ref1 = motionProfiles[3]+p1On;//Ref1Var[k] * (180 / PI);
      Ref2 = motionProfiles[6]+p2On;//Ref2Var[k] * (180 / PI);
      Ref3 = motionProfiles[9];//Ref3Var[k] * (180 / PI);

      Serial.print("Ref 1: ");
      Serial.print(Ref0);
      Serial.print(" Ref 2: ");
      Serial.print(Ref1);
      Serial.print(" Ref 3: ");
      Serial.print(Ref2);
      Serial.print(" Ref 4: ");
      Serial.println(Ref3);

      //Se realiza el calculo de los torques requeridos
      //torque(Ref0Var[k], Ref1Var[k], Ref2Var[k], vq1[k], vq2[k], vq3[k], aq1[k], aq2[k], aq3[k]);
      torque(position0 * (PI / 180), position1 * (PI / 180), position2 * (PI / 180), motionProfiles[1] * (PI / 180), motionProfiles[4] * (PI / 180), motionProfiles[7] * (PI / 180), motionProfiles[2] * (PI / 180), motionProfiles[5] * (PI / 180), motionProfiles[8] * (PI / 180));
      float cmdTau0 = tau1 * 255 / 0.73549875;
      float cmdTau1 = tau2 * 255 / 3.309744375;
      float cmdTau2 = tau3 * 255 / 0.73549875;

      //Se suman las dos señales de control
      float Cmd0 = cmdPD0+ cmdTau0;
      float Cmd1 = cmdPD1+ cmdTau1;
      float Cmd2 = cmdPD2 + cmdTau2;

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
        sentido2 = false;
      } else {
        sentido2 = true;
      }

      //Se obtiene el valor de cmd positivo
      Cmd0 = abs(Cmd0);
      Cmd1 = abs(Cmd1);
      Cmd2 = abs(Cmd2);

      //Función de saturación
      float CmdLim0 = min(max(Cmd0, 0), pwmMax);  // Saturated Control Output
      float CmdLim1 = min(max(Cmd1, 0), pwmMax);  // Saturated Control Output
      float CmdLim2 = min(max(Cmd2, 0), pwmMax);  // Saturated Control Output

      //Ciclos útil de PWM
      pwmDuty0 = int(CmdLim0);
      pwmDuty1 = int(CmdLim1);
      pwmDuty2 = int(CmdLim2);

      //Serial.print(pwmDuty0);
      //Serial.print(" PWM 2: ");
      //Serial.print(pwmDuty1);
      //Serial.print(" PWM 3: ");
      //Serial.println(pwmDuty2);


      //Solo ejecuta cuando hayan pasado 5 segundos
      if (currentMillis >= 5000) {
        setMotor(0, pwmDuty0, sentido0);
        setMotor(1, pwmDuty1, sentido1);
        setMotor(2, pwmDuty2, sentido2);
        myservo.write(Ref3);
      }
    }
  }
}


void torque(float t1, float t2, float t3, float tt1, float tt2, float tt3, float ttt1, float ttt2, float ttt3) {

  float s1 = sin(t1);
  float s2 = sin(t2);
  float s3 = sin(t3);
  float c1 = cos(t1);
  float c2 = cos(t2);
  float c3 = cos(t3);

  float t11 = (11 * ttt1) / 2000 + (279 * ((899 * c2 * s1) / 5000 + (537 * c2 * c3 * s1) / 10000 - (537 * s1 * s2 * s3) / 10000) * ((899 * ttt1 * c2 * s1) / 5000 + (899 * ttt2 * c1 * s2) / 5000 + (899 * tt1 * tt1 * c1 * c2) / 5000 + (899 * tt2 * tt2 * c1 * c2) / 5000 - (899 * tt1 * tt2 * s1 * s2) / 2500 + (537 * ttt1 * c2 * c3 * s1) / 10000 + (537 * ttt2 * c1 * c2 * s3) / 10000 + (537 * ttt2 * c1 * c3 * s2) / 10000 + (537 * ttt3 * c1 * c2 * s3) / 10000 + (537 * ttt3 * c1 * c3 * s2) / 10000 - (537 * ttt1 * s1 * s2 * s3) / 10000 + (537 * tt1 * tt1 * c1 * c2 * c3) / 10000 + (537 * tt2 * tt2 * c1 * c2 * c3) / 10000 + (537 * tt3 * tt3 * c1 * c2 * c3) / 10000 - (537 * tt1 * tt1 * c1 * s2 * s3) / 10000 - (537 * tt2 * tt2 * c1 * s2 * s3) / 10000 - (537 * tt3 * tt3 * c1 * s2 * s3) / 10000 + (537 * tt2 * tt3 * c1 * c2 * c3) / 5000 - (537 * tt1 * tt2 * c2 * s1 * s3) / 5000 - (537 * tt1 * tt2 * c3 * s1 * s2) / 5000 - (537 * tt1 * tt3 * c2 * s1 * s3) / 5000 - (537 * tt1 * tt3 * c3 * s1 * s2) / 5000 - (537 * tt2 * tt3 * c1 * s2 * s3) / 5000)) / 2000;
  float t12 = -(279 * ((899 * c1 * c2) / 5000 + (537 * c1 * c2 * c3) / 10000 - (537 * c1 * s2 * s3) / 10000) * ((899 * ttt2 * s1 * s2) / 5000 - (899 * ttt1 * c1 * c2) / 5000 + (899 * tt1 * tt1 * c2 * s1) / 5000 + (899 * tt2 * tt2 * c2 * s1) / 5000 + (899 * tt1 * tt2 * c1 * s2) / 2500 - (537 * ttt1 * c1 * c2 * c3) / 10000 + (537 * ttt1 * c1 * s2 * s3) / 10000 + (537 * ttt2 * c2 * s1 * s3) / 10000 + (537 * ttt2 * c3 * s1 * s2) / 10000 + (537 * ttt3 * c2 * s1 * s3) / 10000 + (537 * ttt3 * c3 * s1 * s2) / 10000 + (537 * tt1 * tt1 * c2 * c3 * s1) / 10000 + (537 * tt2 * tt2 * c2 * c3 * s1) / 10000 + (537 * tt3 * tt3 * c2 * c3 * s1) / 10000 - (537 * tt1 * tt1 * s1 * s2 * s3) / 10000 - (537 * tt2 * tt2 * s1 * s2 * s3) / 10000 - (537 * tt3 * tt3 * s1 * s2 * s3) / 10000 + (537 * tt1 * tt2 * c1 * c2 * s3) / 5000 + (537 * tt1 * tt2 * c1 * c3 * s2) / 5000 + (537 * tt1 * tt3 * c1 * c2 * s3) / 5000 + (537 * tt1 * tt3 * c1 * c3 * s2) / 5000 + (537 * tt2 * tt3 * c2 * c3 * s1) / 5000 - (537 * tt2 * tt3 * s1 * s2 * s3) / 5000)) / 2000;
  float t13 = (279 * ((899 * c2 * s1) / 5000 - (537 * s1 * s2 * s3) / 10000 + (537 * c2 * c3 * s1) / 10000) * ((899 * c1 * c2 * tt1 * tt1) / 5000 + (899 * c1 * c2 * tt2 * tt2) / 5000 + (899 * c2 * s1 * ttt1) / 5000 + (899 * c1 * s2 * ttt2) / 5000 - (537 * s1 * s2 * s3 * ttt1) / 10000 + (537 * c1 * c2 * c3 * tt1 * tt1) / 10000 + (537 * c1 * c2 * c3 * tt2 * tt2) / 10000 + (537 * c1 * c2 * c3 * tt3 * tt3) / 10000 - (537 * c1 * s2 * s3 * tt1 * tt1) / 10000 - (537 * c1 * s2 * s3 * tt2 * tt2) / 10000 - (537 * c1 * s2 * s3 * tt3 * tt3) / 10000 - (899 * s1 * s2 * tt1 * tt2) / 2500 + (537 * c2 * c3 * s1 * ttt1) / 10000 + (537 * c1 * c2 * s3 * ttt2) / 10000 + (537 * c1 * c3 * s2 * ttt2) / 10000 + (537 * c1 * c2 * s3 * ttt3) / 10000 + (537 * c1 * c3 * s2 * ttt3) / 10000 + (537 * c1 * c2 * c3 * tt2 * tt3) / 5000 - (537 * c2 * s1 * s3 * tt1 * tt2) / 5000 - (537 * c3 * s1 * s2 * tt1 * tt2) / 5000 - (537 * c2 * s1 * s3 * tt1 * tt3) / 5000 - (537 * c3 * s1 * s2 * tt1 * tt3) / 5000 - (537 * c1 * s2 * s3 * tt2 * tt3) / 5000)) / 2000;
  float t14 = -(279 * ((899 * c1 * c2) / 5000 - (537 * c1 * s2 * s3) / 10000 + (537 * c1 * c2 * c3) / 10000) * ((899 * c2 * s1 * tt1 * tt1) / 5000 + (899 * c2 * s1 * tt2 * tt2) / 5000 - (899 * c1 * c2 * ttt1) / 5000 + (899 * s1 * s2 * ttt2) / 5000 + (537 * c1 * s2 * s3 * ttt1) / 10000 + (537 * c2 * s1 * s3 * ttt2) / 10000 + (537 * c3 * s1 * s2 * ttt2) / 10000 + (537 * c2 * s1 * s3 * ttt3) / 10000 + (537 * c3 * s1 * s2 * ttt3) / 10000 + (537 * c2 * c3 * s1 * tt1 * tt1) / 10000 + (537 * c2 * c3 * s1 * tt2 * tt2) / 10000 + (537 * c2 * c3 * s1 * tt3 * tt3) / 10000 - (537 * s1 * s2 * s3 * tt1 * tt1) / 10000 - (537 * s1 * s2 * s3 * tt2 * tt2) / 10000 - (537 * s1 * s2 * s3 * tt3 * tt3) / 10000 + (899 * c1 * s2 * tt1 * tt2) / 2500 - (537 * c1 * c2 * c3 * ttt1) / 10000 + (537 * c1 * c2 * s3 * tt1 * tt2) / 5000 + (537 * c1 * c3 * s2 * tt1 * tt2) / 5000 + (537 * c1 * c2 * s3 * tt1 * tt3) / 5000 + (537 * c1 * c3 * s2 * tt1 * tt3) / 5000 + (537 * c2 * c3 * s1 * tt2 * tt3) / 5000 - (537 * s1 * s2 * s3 * tt2 * tt3) / 5000)) / 2000;
  float t15 = -(3819 * c1 * c2 * ((201 * c2 * s1 * tt1 * tt1) / 2500 + (201 * c1 * s2 * tt1 * tt2) / 1250 + (201 * c2 * s1 * tt2 * tt2) / 2500 - (201 * ttt1 * c1 * c2) / 2500 + (201 * ttt2 * s1 * s2) / 2500)) / 625000 + (3819 * c2 * s1 * ((201 * c1 * c2 * tt1 * tt1) / 2500 - (201 * s1 * s2 * tt1 * tt2) / 1250 + (201 * c1 * c2 * tt2 * tt2) / 2500 + (201 * ttt1 * c2 * s1) / 2500 + (201 * ttt2 * c1 * s2) / 2500)) / 625000 - (3819 * c1 * c2 * ((201 * c2 * s1 * tt1 * tt1) / 2500 + (201 * c1 * s2 * tt1 * tt2) / 1250 + (201 * c2 * s1 * tt2 * tt2) / 2500 - (201 * c1 * c2 * ttt1) / 2500 + (201 * s1 * s2 * ttt2) / 2500)) / 625000 + (3819 * c2 * s1 * ((201 * c1 * c2 * tt1 * tt1) / 2500 - (201 * s1 * s2 * tt1 * tt2) / 1250 + (201 * c1 * c2 * tt2 * tt2) / 2500 + (201 * c2 * s1 * ttt1) / 2500 + (201 * c1 * s2 * ttt2) / 2500)) / 625000;
  float t16 = (3819 * c1 * c2 * tt1 * ((201 * tt1 * c2 * s1) / 2500 + (201 * tt2 * c1 * s2) / 2500)) / 625000 - (3819 * c2 * s1 * tt1 * ((201 * tt1 * c1 * c2) / 2500 - (201 * tt2 * s1 * s2) / 2500)) / 625000 - (3819 * c1 * s2 * tt2 * ((201 * tt1 * c1 * c2) / 2500 - (201 * tt2 * s1 * s2) / 2500)) / 625000 - (3819 * s1 * s2 * tt2 * ((201 * tt1 * c2 * s1) / 2500 + (201 * tt2 * c1 * s2) / 2500)) / 625000 + (3819 * tt1 * c1 * c2 * ((201 * c2 * s1 * tt1) / 2500 + (201 * c1 * s2 * tt2) / 2500)) / 625000 - (3819 * tt1 * c2 * s1 * ((201 * c1 * c2 * tt1) / 2500 - (201 * s1 * s2 * tt2) / 2500)) / 625000 - (3819 * tt2 * c1 * s2 * ((201 * c1 * c2 * tt1) / 2500 - (201 * s1 * s2 * tt2) / 2500)) / 625000 - (3819 * tt2 * s1 * s2 * ((201 * c2 * s1 * tt1) / 2500 + (201 * c1 * s2 * tt2) / 2500)) / 625000;

  tau1 = t11 + t12 + t13 + t14 + t15 + t16;

  float t21 = (0.0007068 * ttt2) + (12239937 * c2) / 20000000 + (146976363 * c2 * c3) / 1000000000 + (279 * ((899 * c1 * s2) / 5000 + (537 * c1 * c2 * s3) / 10000 + (537 * c1 * c3 * s2) / 10000) * ((899 * ttt1 * c2 * s1) / 5000 + (899 * ttt2 * c1 * s2) / 5000 + (899 * tt1 * tt1 * c1 * c2) / 5000 + (899 * tt2 * tt2 * c1 * c2) / 5000 - (899 * tt1 * tt2 * s1 * s2) / 2500 + (537 * ttt1 * c2 * c3 * s1) / 10000 + (537 * ttt2 * c1 * c2 * s3) / 10000 + (537 * ttt2 * c1 * c3 * s2) / 10000 + (537 * ttt3 * c1 * c2 * s3) / 10000 + (537 * ttt3 * c1 * c3 * s2) / 10000 - (537 * ttt1 * s1 * s2 * s3) / 10000 + (537 * tt1 * tt1 * c1 * c2 * c3) / 10000 + (537 * tt2 * tt2 * c1 * c2 * c3) / 10000 + (537 * tt3 * tt3 * c1 * c2 * c3) / 10000 - (537 * tt1 * tt1 * c1 * s2 * s3) / 10000 - (537 * tt2 * tt2 * c1 * s2 * s3) / 10000 - (537 * tt3 * tt3 * c1 * s2 * s3) / 10000 + (537 * tt2 * tt3 * c1 * c2 * c3) / 5000 - (537 * tt1 * tt2 * c2 * s1 * s3) / 5000 - (537 * tt1 * tt2 * c3 * s1 * s2) / 5000 - (537 * tt1 * tt3 * c2 * s1 * s3) / 5000 - (537 * tt1 * tt3 * c3 * s1 * s2) / 5000 - (537 * tt2 * tt3 * c1 * s2 * s3) / 5000)) / 2000;
  float t22 = (279 * ((899 * s1 * s2) / 5000 + (537 * c2 * s1 * s3) / 10000 + (537 * c3 * s1 * s2) / 10000) * ((899 * c2 * s1 * tt1 * tt1) / 5000 + (899 * c2 * s1 * tt2 * tt2) / 5000 - (899 * c1 * c2 * ttt1) / 5000 + (899 * s1 * s2 * ttt2) / 5000 + (537 * c1 * s2 * s3 * ttt1) / 10000 + (537 * c2 * s1 * s3 * ttt2) / 10000 + (537 * c3 * s1 * s2 * ttt2) / 10000 + (537 * c2 * s1 * s3 * ttt3) / 10000 + (537 * c3 * s1 * s2 * ttt3) / 10000 + (537 * c2 * c3 * s1 * tt1 * tt1) / 10000 + (537 * c2 * c3 * s1 * tt2 * tt2) / 10000 + (537 * c2 * c3 * s1 * tt3 * tt3) / 10000 - (537 * s1 * s2 * s3 * tt1 * tt1) / 10000 - (537 * s1 * s2 * s3 * tt2 * tt2) / 10000 - (537 * s1 * s2 * s3 * tt3 * tt3) / 10000 + (899 * c1 * s2 * tt1 * tt2) / 2500 - (537 * c1 * c2 * c3 * ttt1) / 10000 + (537 * c1 * c2 * s3 * tt1 * tt2) / 5000 + (537 * c1 * c3 * s2 * tt1 * tt2) / 5000 + (537 * c1 * c2 * s3 * tt1 * tt3) / 5000 + (537 * c1 * c3 * s2 * tt1 * tt3) / 5000 + (537 * c2 * c3 * s1 * tt2 * tt3) / 5000 - (537 * s1 * s2 * s3 * tt2 * tt3) / 5000)) / 2000;
  float t23 = -(146976363 * s2 * s3) / 1000000000 - (279 * ((899 * c2) / 5000 + (537 * c2 * c3) / 10000 - (537 * s2 * s3) / 10000) * ((899 * tt2 * tt2 * s2) / 5000 - (899 * ttt2 * c2) / 5000 - (537 * ttt2 * c2 * c3) / 10000 - (537 * ttt3 * c2 * c3) / 10000 + (537 * ttt2 * s2 * s3) / 10000 + (537 * ttt3 * s2 * s3) / 10000 + (537 * tt2 * tt2 * c2 * s3) / 10000 + (537 * tt2 * tt2 * c3 * s2) / 10000 + (537 * tt3 * tt3 * c2 * s3) / 10000 + (537 * tt3 * tt3 * c3 * s2) / 10000 + (537 * tt2 * tt3 * c2 * s3) / 5000 + (537 * tt2 * tt3 * c3 * s2) / 5000)) / 2000;
  float t24 = -(279 * ((899 * c2) / 5000 + (537 * c2 * c3) / 10000 - (537 * s2 * s3) / 10000) * ((899 * s2 * tt2 * tt2) / 5000 - (899 * c2 * ttt2) / 5000 + (537 * c2 * s3 * tt2 * tt2) / 10000 + (537 * c3 * s2 * tt2 * tt2) / 10000 + (537 * c2 * s3 * tt3 * tt3) / 10000 + (537 * c3 * s2 * tt3 * tt3) / 10000 - (537 * c2 * c3 * ttt2) / 10000 - (537 * c2 * c3 * ttt3) / 10000 + (537 * s2 * s3 * ttt2) / 10000 + (537 * s2 * s3 * ttt3) / 10000 + (537 * c2 * s3 * tt2 * tt3) / 5000 + (537 * c3 * s2 * tt2 * tt3) / 5000)) / 2000;
  float t25 = (279 * ((899 * c1 * s2) / 5000 + (537 * c1 * c2 * s3) / 10000 + (537 * c1 * c3 * s2) / 10000) * ((899 * c1 * c2 * tt1 * tt1) / 5000 + (899 * c1 * c2 * tt2 * tt2) / 5000 + (899 * c2 * s1 * ttt1) / 5000 + (899 * c1 * s2 * ttt2) / 5000 - (537 * s1 * s2 * s3 * ttt1) / 10000 + (537 * c1 * c2 * c3 * tt1 * tt1) / 10000 + (537 * c1 * c2 * c3 * tt2 * tt2) / 10000 + (537 * c1 * c2 * c3 * tt3 * tt3) / 10000 - (537 * c1 * s2 * s3 * tt1 * tt1) / 10000 - (537 * c1 * s2 * s3 * tt2 * tt2) / 10000 - (537 * c1 * s2 * s3 * tt3 * tt3) / 10000 - (899 * s1 * s2 * tt1 * tt2) / 2500 + (537 * c2 * c3 * s1 * ttt1) / 10000 + (537 * c1 * c2 * s3 * ttt2) / 10000 + (537 * c1 * c3 * s2 * ttt2) / 10000 + (537 * c1 * c2 * s3 * ttt3) / 10000 + (537 * c1 * c3 * s2 * ttt3) / 10000 + (537 * c1 * c2 * c3 * tt2 * tt3) / 5000 - (537 * c2 * s1 * s3 * tt1 * tt2) / 5000 - (537 * c3 * s1 * s2 * tt1 * tt2) / 5000 - (537 * c2 * s1 * s3 * tt1 * tt3) / 5000 - (537 * c3 * s1 * s2 * tt1 * tt3) / 5000 - (537 * c1 * s2 * s3 * tt2 * tt3) / 5000)) / 2000;
  float t26 = (279 * ((899 * s1 * s2) / 5000 + (537 * c2 * s1 * s3) / 10000 + (537 * c3 * s1 * s2) / 10000) * ((899 * ttt2 * s1 * s2) / 5000 - (899 * ttt1 * c1 * c2) / 5000 + (899 * tt1 * tt1 * c2 * s1) / 5000 + (899 * tt2 * tt2 * c2 * s1) / 5000 + (899 * tt1 * tt2 * c1 * s2) / 2500 - (537 * ttt1 * c1 * c2 * c3) / 10000 + (537 * ttt1 * c1 * s2 * s3) / 10000 + (537 * ttt2 * c2 * s1 * s3) / 10000 + (537 * ttt2 * c3 * s1 * s2) / 10000 + (537 * ttt3 * c2 * s1 * s3) / 10000 + (537 * ttt3 * c3 * s1 * s2) / 10000 + (537 * tt1 * tt1 * c2 * c3 * s1) / 10000 + (537 * tt2 * tt2 * c2 * c3 * s1) / 10000 + (537 * tt3 * tt3 * c2 * c3 * s1) / 10000 - (537 * tt1 * tt1 * s1 * s2 * s3) / 10000 - (537 * tt2 * tt2 * s1 * s2 * s3) / 10000 - (537 * tt3 * tt3 * s1 * s2 * s3) / 10000 + (537 * tt1 * tt2 * c1 * c2 * s3) / 5000 + (537 * tt1 * tt2 * c1 * c3 * s2) / 5000 + (537 * tt1 * tt3 * c1 * c2 * s3) / 5000 + (537 * tt1 * tt3 * c1 * c3 * s2) / 5000 + (537 * tt2 * tt3 * c2 * c3 * s1) / 5000 - (537 * tt2 * tt3 * s1 * s2 * s3) / 5000)) / 2000;
  float t27 = (19 * ((201 * tt1 * c1 * c2) / 2500 - (201 * tt2 * s1 * s2) / 2500) * ((201 * c1 * s2 * tt1) / 2500 + (201 * c2 * s1 * tt2) / 2500)) / 250 - (19 * ((201 * tt2 * c1 * c2) / 2500 - (201 * tt1 * s1 * s2) / 2500) * ((201 * c2 * s1 * tt1) / 2500 + (201 * c1 * s2 * tt2) / 2500)) / 250 + (19 * ((201 * tt1 * c1 * s2) / 2500 + (201 * tt2 * c2 * s1) / 2500) * ((201 * c1 * c2 * tt1) / 2500 - (201 * s1 * s2 * tt2) / 2500)) / 250 - (19 * ((201 * tt1 * c2 * s1) / 2500 + (201 * tt2 * c1 * s2) / 2500) * ((201 * c1 * c2 * tt2) / 2500 - (201 * s1 * s2 * tt1) / 2500)) / 250 + (3819 * c1 * s2 * ((201 * c1 * c2 * tt1 * tt1) / 2500 - (201 * s1 * s2 * tt1 * tt2) / 1250 + (201 * c1 * c2 * tt2 * tt2) / 2500 + (201 * ttt1 * c2 * s1) / 2500 + (201 * ttt2 * c1 * s2) / 2500)) / 625000;
  float t28 = (767619 * c2 * c2 * ttt2) / 1562500000 - (767619 * tt2 * tt2 * c2 * s2) / 1562500000 + (3819 * s1 * s2 * ((201 * c2 * s1 * tt1 * tt1) / 2500 + (201 * c1 * s2 * tt1 * tt2) / 1250 + (201 * c2 * s1 * tt2 * tt2) / 2500 - (201 * ttt1 * c1 * c2) / 2500 + (201 * ttt2 * s1 * s2) / 2500)) / 625000 + (3819 * c1 * s2 * ((201 * c1 * c2 * tt1 * tt1) / 2500 - (201 * s1 * s2 * tt1 * tt2) / 1250 + (201 * c1 * c2 * tt2 * tt2) / 2500 + (201 * c2 * s1 * ttt1) / 2500 + (201 * c1 * s2 * ttt2) / 2500)) / 625000;
  float t29 = -(767619 * s2 * c2 * tt2 * tt2) / 1562500000 + (3819 * s1 * s2 * ((201 * c2 * s1 * tt1 * tt1) / 2500 + (201 * c1 * s2 * tt1 * tt2) / 1250 + (201 * c2 * s1 * tt2 * tt2) / 2500 - (201 * c1 * c2 * ttt1) / 2500 + (201 * s1 * s2 * ttt2) / 2500)) / 625000 + (767619 * ttt2 * c2 * c2) / 1562500000 + (3819 * c1 * c2 * tt2 * ((201 * tt1 * c2 * s1) / 2500 + (201 * tt2 * c1 * s2) / 2500)) / 625000 - (3819 * c1 * s2 * tt1 * ((201 * tt1 * c1 * c2) / 2500 - (201 * tt2 * s1 * s2) / 2500)) / 625000;
  float t210 = -(3819 * c2 * s1 * tt2 * ((201 * tt1 * c1 * c2) / 2500 - (201 * tt2 * s1 * s2) / 2500)) / 625000 - (3819 * s1 * s2 * tt1 * ((201 * tt1 * c2 * s1) / 2500 + (201 * tt2 * c1 * s2) / 2500)) / 625000 + (3819 * tt2 * c1 * c2 * ((201 * c2 * s1 * tt1) / 2500 + (201 * c1 * s2 * tt2) / 2500)) / 625000 - (3819 * tt1 * c1 * s2 * ((201 * c1 * c2 * tt1) / 2500 - (201 * s1 * s2 * tt2) / 2500)) / 625000 - (3819 * tt2 * c2 * s1 * ((201 * c1 * c2 * tt1) / 2500 - (201 * s1 * s2 * tt2) / 2500)) / 625000 - (3819 * tt1 * s1 * s2 * ((201 * c2 * s1 * tt1) / 2500 + (201 * c1 * s2 * tt2) / 2500)) / 625000;

  tau2 = t21 + t22 + t23 + t24 + t25 + t26 + t27 + t28 + t29 + t210;

  float t31 = (11 * ttt3) / 10000 + (146976363 * c2 * c3) / 1000000000 + (279 * ((537 * c1 * c2 * s3) / 10000 + (537 * c1 * c3 * s2) / 10000) * ((899 * ttt1 * c2 * s1) / 5000 + (899 * ttt2 * c1 * s2) / 5000 + (899 * tt1 * tt1 * c1 * c2) / 5000 + (899 * tt2 * tt2 * c1 * c2) / 5000 - (899 * tt1 * tt2 * s1 * s2) / 2500 + (537 * ttt1 * c2 * c3 * s1) / 10000 + (537 * ttt2 * c1 * c2 * s3) / 10000 + (537 * ttt2 * c1 * c3 * s2) / 10000 + (537 * ttt3 * c1 * c2 * s3) / 10000 + (537 * ttt3 * c1 * c3 * s2) / 10000 - (537 * ttt1 * s1 * s2 * s3) / 10000 + (537 * tt1 * tt1 * c1 * c2 * c3) / 10000 + (537 * tt2 * tt2 * c1 * c2 * c3) / 10000 + (537 * tt3 * tt3 * c1 * c2 * c3) / 10000 - (537 * tt1 * tt1 * c1 * s2 * s3) / 10000 - (537 * tt2 * tt2 * c1 * s2 * s3) / 10000 - (537 * tt3 * tt3 * c1 * s2 * s3) / 10000 + (537 * tt2 * tt3 * c1 * c2 * c3) / 5000 - (537 * tt1 * tt2 * c2 * s1 * s3) / 5000 - (537 * tt1 * tt2 * c3 * s1 * s2) / 5000 - (537 * tt1 * tt3 * c2 * s1 * s3) / 5000 - (537 * tt1 * tt3 * c3 * s1 * s2) / 5000 - (537 * tt2 * tt3 * c1 * s2 * s3) / 5000)) / 2000;
  float t32 = -(146976363 * s2 * s3) / 1000000000 - (279 * ((537 * c2 * c3) / 10000 - (537 * s2 * s3) / 10000) * ((899 * s2 * tt2 * tt2) / 5000 - (899 * c2 * ttt2) / 5000 + (537 * c2 * s3 * tt2 * tt2) / 10000 + (537 * c3 * s2 * tt2 * tt2) / 10000 + (537 * c2 * s3 * tt3 * tt3) / 10000 + (537 * c3 * s2 * tt3 * tt3) / 10000 - (537 * c2 * c3 * ttt2) / 10000 - (537 * c2 * c3 * ttt3) / 10000 + (537 * s2 * s3 * ttt2) / 10000 + (537 * s2 * s3 * ttt3) / 10000 + (537 * c2 * s3 * tt2 * tt3) / 5000 + (537 * c3 * s2 * tt2 * tt3) / 5000)) / 2000;
  float t33 = (279 * ((537 * c1 * c2 * s3) / 10000 + (537 * c1 * c3 * s2) / 10000) * ((899 * c1 * c2 * tt1 * tt1) / 5000 + (899 * c1 * c2 * tt2 * tt2) / 5000 + (899 * c2 * s1 * ttt1) / 5000 + (899 * c1 * s2 * ttt2) / 5000 - (537 * s1 * s2 * s3 * ttt1) / 10000 + (537 * c1 * c2 * c3 * tt1 * tt1) / 10000 + (537 * c1 * c2 * c3 * tt2 * tt2) / 10000 + (537 * c1 * c2 * c3 * tt3 * tt3) / 10000 - (537 * c1 * s2 * s3 * tt1 * tt1) / 10000 - (537 * c1 * s2 * s3 * tt2 * tt2) / 10000 - (537 * c1 * s2 * s3 * tt3 * tt3) / 10000 - (899 * s1 * s2 * tt1 * tt2) / 2500 + (537 * c2 * c3 * s1 * ttt1) / 10000 + (537 * c1 * c2 * s3 * ttt2) / 10000 + (537 * c1 * c3 * s2 * ttt2) / 10000 + (537 * c1 * c2 * s3 * ttt3) / 10000 + (537 * c1 * c3 * s2 * ttt3) / 10000 + (537 * c1 * c2 * c3 * tt2 * tt3) / 5000 - (537 * c2 * s1 * s3 * tt1 * tt2) / 5000 - (537 * c3 * s1 * s2 * tt1 * tt2) / 5000 - (537 * c2 * s1 * s3 * tt1 * tt3) / 5000 - (537 * c3 * s1 * s2 * tt1 * tt3) / 5000 - (537 * c1 * s2 * s3 * tt2 * tt3) / 5000)) / 2000;
  float t34 = (279 * ((537 * c2 * s1 * s3) / 10000 + (537 * c3 * s1 * s2) / 10000) * ((899 * ttt2 * s1 * s2) / 5000 - (899 * ttt1 * c1 * c2) / 5000 + (899 * tt1 * tt1 * c2 * s1) / 5000 + (899 * tt2 * tt2 * c2 * s1) / 5000 + (899 * tt1 * tt2 * c1 * s2) / 2500 - (537 * ttt1 * c1 * c2 * c3) / 10000 + (537 * ttt1 * c1 * s2 * s3) / 10000 + (537 * ttt2 * c2 * s1 * s3) / 10000 + (537 * ttt2 * c3 * s1 * s2) / 10000 + (537 * ttt3 * c2 * s1 * s3) / 10000 + (537 * ttt3 * c3 * s1 * s2) / 10000 + (537 * tt1 * tt1 * c2 * c3 * s1) / 10000 + (537 * tt2 * tt2 * c2 * c3 * s1) / 10000 + (537 * tt3 * tt3 * c2 * c3 * s1) / 10000 - (537 * tt1 * tt1 * s1 * s2 * s3) / 10000 - (537 * tt2 * tt2 * s1 * s2 * s3) / 10000 - (537 * tt3 * tt3 * s1 * s2 * s3) / 10000 + (537 * tt1 * tt2 * c1 * c2 * s3) / 5000 + (537 * tt1 * tt2 * c1 * c3 * s2) / 5000 + (537 * tt1 * tt3 * c1 * c2 * s3) / 5000 + (537 * tt1 * tt3 * c1 * c3 * s2) / 5000 + (537 * tt2 * tt3 * c2 * c3 * s1) / 5000 - (537 * tt2 * tt3 * s1 * s2 * s3) / 5000)) / 2000;
  float t35 = -(279 * ((537 * c2 * c3) / 10000 - (537 * s2 * s3) / 10000) * ((899 * tt2 * tt2 * s2) / 5000 - (899 * ttt2 * c2) / 5000 - (537 * ttt2 * c2 * c3) / 10000 - (537 * ttt3 * c2 * c3) / 10000 + (537 * ttt2 * s2 * s3) / 10000 + (537 * ttt3 * s2 * s3) / 10000 + (537 * tt2 * tt2 * c2 * s3) / 10000 + (537 * tt2 * tt2 * c3 * s2) / 10000 + (537 * tt3 * tt3 * c2 * s3) / 10000 + (537 * tt3 * tt3 * c3 * s2) / 10000 + (537 * tt2 * tt3 * c2 * s3) / 5000 + (537 * tt2 * tt3 * c3 * s2) / 5000)) / 2000;
  float t36 = (279 * ((537 * c2 * s1 * s3) / 10000 + (537 * c3 * s1 * s2) / 10000) * ((899 * c2 * s1 * tt1 * tt1) / 5000 + (899 * c2 * s1 * tt2 * tt2) / 5000 - (899 * c1 * c2 * ttt1) / 5000 + (899 * s1 * s2 * ttt2) / 5000 + (537 * c1 * s2 * s3 * ttt1) / 10000 + (537 * c2 * s1 * s3 * ttt2) / 10000 + (537 * c3 * s1 * s2 * ttt2) / 10000 + (537 * c2 * s1 * s3 * ttt3) / 10000 + (537 * c3 * s1 * s2 * ttt3) / 10000 + (537 * c2 * c3 * s1 * tt1 * tt1) / 10000 + (537 * c2 * c3 * s1 * tt2 * tt2) / 10000 + (537 * c2 * c3 * s1 * tt3 * tt3) / 10000 - (537 * s1 * s2 * s3 * tt1 * tt1) / 10000 - (537 * s1 * s2 * s3 * tt2 * tt2) / 10000 - (537 * s1 * s2 * s3 * tt3 * tt3) / 10000 + (899 * c1 * s2 * tt1 * tt2) / 2500 - (537 * c1 * c2 * c3 * ttt1) / 10000 + (537 * c1 * c2 * s3 * tt1 * tt2) / 5000 + (537 * c1 * c3 * s2 * tt1 * tt2) / 5000 + (537 * c1 * c2 * s3 * tt1 * tt3) / 5000 + (537 * c1 * c3 * s2 * tt1 * tt3) / 5000 + (537 * c2 * c3 * s1 * tt2 * tt3) / 5000 - (537 * s1 * s2 * s3 * tt2 * tt3) / 5000)) / 2000;

  tau3 = t21 + t32 + t33 + t34 + t35 + t36;
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


void sendCommand(const char * command){
  //Serial.print("Command send :");
  //Serial.println(command);
  //Serial2.println(command);
  delay(100);

  char reply[100];
  int i = 0;
  while(Serial2.available()){
    reply[i] = Serial2.read();
    i += 1;
  }
  reply[i] = '\0';
  //Serial.print(reply);
  //Serial.println("Reply end");
}

void readSerial(){
  String replyStr = "";
  char reply[200];
  int i = 0;
  while(Serial2.available()){
    reply[i] = Serial2.read();
    i = i+1;
  }
  bool startFlagIsDetected = false;
  String startFlag = "1010101000";
  String endFlag = "00100011";
  int startValuesFound = 0;
  int endValuesFound = 0;
  bool foundFirstOne = false;
  String data = "";
  for (int i=0; i<sizeof reply/sizeof reply[0]; i++) {
    //Serial.print(reply[i], BIN);
    String binaryString = "";
    for(int j = 0, mask = 128; j < 8; j++, mask = mask >> 1){
        bool bitValue;
        if ((reply[i] & mask) && !foundFirstOne){
            foundFirstOne = true;
            bitValue = true;
        }else if (reply[i] & mask){
            binaryString += "1";
            bitValue = true;
        }else if (foundFirstOne){
            binaryString += "0";
            bitValue = false;
        }
        if (bitValue){
          binaryString += oneT;
          if (startFlagIsDetected){
            data += oneT;
          }
          if (startFlag[startValuesFound] == oneT && !startFlagIsDetected) {
            startValuesFound += 1;
          }else{
            startValuesFound = 0;
          }
          if (endFlag[endValuesFound] == oneT && startFlagIsDetected) {
            endValuesFound += 1;
          }else{
            endValuesFound = 0;
          }
        }else {
          if (startFlagIsDetected){
            data += zeroT;
          }
          binaryString += zeroT;
          if (startFlag[startValuesFound] == zeroT && !startFlagIsDetected) {
            startValuesFound += 1;
          }else{
            startValuesFound = 0;
          }
          if (endFlag[endValuesFound] == zeroT && startFlagIsDetected) {
            endValuesFound += 1;
          }else{
            endValuesFound = 0;
          }
        }
        if (startValuesFound ==  6){
          startFlagIsDetected = true;
          //Serial.print("Flag");
          startValuesFound = 0;
        }
        if (endValuesFound == 8){
          //data = data.substring(0, data.length() - 8);
          sendDataToController(data);
          //Serial.println("End");
          endValuesFound = 0;
          startFlagIsDetected = false;
        }
    }
    
    if (!foundFirstOne) {
      binaryString = "0";
    }
    replyStr += binaryString;
  }
  //Serial.println(replyStr);
  
}

void sendDataToController(String data) {
  String configurationData = data.substring(6,16);
  //Serial.print(configurationData);
  String names[10] = {" 1p: "," 1v: "," 1a: "," 2p: "," 2v: "," 2a: "," 3p: "," 3v: "," 3a: "," 4p: "};
  int numbersSaved = 0;
  for (int i = 0; i<10; i++){
    if(configurationData[i] == oneT){
      //Serial.print(names[i]);
      String s = data.substring(16+16*numbersSaved,32+16*numbersSaved);
      int value = 0;
      for(int j = 0; j < s.length(); j++){
        //Serial.print("v");
        //Serial.print(value);
        //Serial.print("INTM");
        //Serial.print((int(s[j])-48));
        //Serial.print("pou");
        //Serial.print(pow(2,s.length()-1-j));
        value += pow(2,s.length()-1-j)*(int(s[j])-48);
      }
      if(abs( ((value-32767) * 0.01)-motionProfiles[i])<10){
        motionProfiles[i] = (value-32767) * 0.01;
      }
      //Serial.print(names[i]);
      //Serial.print(s);
      //Serial.print(" ");
      //Serial.print(value);
      //Serial.print(" ");
      //Serial.print(motionProfiles[i]);
      numbersSaved += 1;
    }
  }
  //Serial.println();
}
