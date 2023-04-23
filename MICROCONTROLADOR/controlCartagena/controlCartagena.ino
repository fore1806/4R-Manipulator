//***************************************************//
//***************************************************//
//*****   Posición y Velocidad de Motor DC      *****//
//*****                                         *****//
//***** by: Sergio Andres Castaño Giraldo       *****//
//***** https://controlautomaticoeducacion.com/ *****//
//*****                                         *****//
//***************************************************//
//***************************************************//

// this library includes the ATOMIC_BLOCK macro.
#include "SimplyAtomic.h"
#include "BTS7960.h"
#include "Encoder.h"

typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

//Objetos de envio y recepcion de datos

//Variables de recepción
FLOATUNION_t RefQ0;
FLOATUNION_t RefQ1;
FLOATUNION_t RefQ2;
FLOATUNION_t RefQ3;

//Variables de Envio
FLOATUNION_t q0M;
FLOATUNION_t q1M;
FLOATUNION_t q2M;
FLOATUNION_t q3M;
FLOATUNION_t Rq0;
FLOATUNION_t Rq1;
FLOATUNION_t Rq2;
FLOATUNION_t Rq3;

// Pines de Control L298N Motor 0
#define RPWM0 7
#define LPWM0 6
#define EN0 5

// Controller BTS7960 -> Motor Hombro
const uint8_t EN1 = 11;  //L_EN = R_EN -> Mismo pin
const uint8_t L_PWM1 = 13;
const uint8_t R_PWM1 = 12;

// Pines de Control L298N Motor 2
#define RPWM2 8
#define LPWM2 9
#define EN2 10

// Se instancia el controlador, Enable usado para los dos sentidos!
BTS7960 motorController(EN1, L_PWM1, R_PWM1);

//Encoder Motor 0
#define ENCODER_A0 2  // Amarillo
#define ENCODER_B0 3  // Verde
Encoder myEnc0(2, 3);

//Encoder Motor 1
#define ENCODER_A1 22  // Amarillo
#define ENCODER_B1 24  // Blanco
Encoder myEnc1(22, 24);

//Encoder Motor 2
#define ENCODER_A2 23  // Amarillo
#define ENCODER_B2 25  // Verde
Encoder myEnc2(23, 25);

//Sentido de los motores
boolean sentido0 = true;
boolean sentido1 = true;
boolean sentido2 = true;

//Constantes
float posicion0;
float posicion1;
float posicion2;

long oldPosition1 = -999;

float rpm0;
float rpm1;
float rpm2;

int value0, dir0 = true;
int value1, dir1 = true;
int value2, dir2 = true;

//Compensación gravitacional

float m1 = 0.163;  //Kg
float l1 = 0.187;  //m
float lc1 = 0.1178;
float I1 = 0.0012;  //Nm/s2

float m2 = 0.1038;  //kg
float l2 = 0.181;   //m
float lc2 = 0.0994;
float I2 = 5.5942e-4;

float g = 9.81;

//Variable global de posición compartida con la interrupción
long theta0 = 0;
long theta1 = 0;
long theta2 = 0;
//pruebas
volatile int theta = 0;
volatile float angulo = 0.0;

//Variable global de pulsos compartida con la interrupción
volatile int pulsos0;
long timeold0 = -999;

volatile int pulsos1 = 0;
long timeold1 = -999;

volatile int pulsos2 = 0;
long timeold2 = -999;

long long previousMillis;

float resolution = 360.0 / (44.0 * 34.02);
float resolutionPololu = 360.0 / 8400.0;
int duracion = -1000;

//Variable Global Velocidad
int vel0 = 0;
int vel1 = 0;
int vel2 = 0;

//Variable Global Posicion
int ang0 = 0;
int ang1 = 0;
int ang2 = 0;

//Variable Global MODO
bool modo = true;

//Constantes de Controlador
float kp1 = 4.63 * 3;
float CmdP = 0;
unsigned int pwmDuty = 0;
int pwmMax = 255;
int pwmMin = 85;
int pwmMin2 = 85;
float E1 = 0;
float Ref1 = 120;

//Constantes de Controlador
float kp2 = 12;  //2.22;//9.65;//0.84;
float CmdP2 = 0;
unsigned int pwmDuty2 = 0;
float E2 = 0;
float Ref2 = -100;

long int cmillis;
long int cmillis2;


float Ts = 4;
float TsC = 1;



void setup() {
  // set timer 1 divisor to  1024 for PWM frequency of 30.64 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;
  Serial.begin(115200);

  RefQ0.number = 0;
  RefQ1.number = 180;
  RefQ2.number = -165;

  while (!Serial) {
    ;  // wait for serial port to connect. Needed for Leonardo only
  }

  //Encoders como entradas
  //pinMode(ENCODER_A0, INPUT);
  //pinMode(ENCODER_B0, INPUT);

  //pinMode(ENCODER_A1, INPUT);
  //pinMode(ENCODER_B1, INPUT);

  //pinMode(ENCODER_A2, INPUT);
  //pinMode(ENCODER_B2, INPUT);

  //Configura Motor base
  digitalWrite(LPWM2, LOW);
  digitalWrite(RPWM2, LOW);
  digitalWrite(EN2, LOW);
  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(EN2, OUTPUT);

  //Configura Motor codo
  digitalWrite(LPWM0, LOW);
  digitalWrite(RPWM0, LOW);
  digitalWrite(EN0, LOW);
  pinMode(LPWM0, OUTPUT);
  pinMode(RPWM0, OUTPUT);
  pinMode(EN0, OUTPUT);

  //Configurar Interrupción
  //timeold0 = 0;
  //attachInterrupt(digitalPinToInterrupt(ENCODER_A0), leerEncoder0, RISING);

  //timeold1 = 0;
  //attachInterrupt(digitalPinToInterrupt(ENCODER_A1), leerEncoder1, RISING);

  //timeold2 = 0;
  //attachInterrupt(digitalPinToInterrupt(ENCODER_A2), leerEncoder2, RISING);
  delay(3000);
}

void loop() {
  //motorController.Enable();
  /*
  if (millis() - cmillis2 > 5) {
    //Recepción
    Recepcion();
    cmillis2 = millis();
  }

  //Controlador
  controlP();

  if (millis() - cmillis > 5) {
    //Envio
    Envio();
    cmillis = millis();
  }*/


  caracterizacion();
  theta0 = myEnc0.read();
  theta1 = myEnc1.read();
  theta2 = myEnc2.read();
  posicion0 = resolution * theta0;
  posicion1 = resolutionPololu * theta1;
  posicion2 = resolution * theta2;
/*  
  Serial.print(theta0);
  Serial.print(", ");
  Serial.print(theta1);
  Serial.print(", ");
  Serial.println(theta2);*/
  Serial.print(posicion0);
  Serial.print(", ");
  Serial.print(posicion1);
  Serial.print(", ");
  Serial.println(posicion2);
  //Serial.println(pulsos1);
}

void caracterizacion() {
  unsigned long currentMillis = millis();
  int vel = 100;
  int vel1 = 40;
  int vel2 = 100;
  int t2 = 5200;
  int t3 = 7000;
  int t4 = 7200;
  if (currentMillis >= 5000 && currentMillis < t2) {
    //setMotor(0, vel, true);
    //setMotor(1, vel1, true);
    setMotor(2, vel2, true);
  } else if (currentMillis >= t2 && currentMillis < t3) {
    setMotor(0, 0, true);
    setMotor(1, 0, true);
    setMotor(2, 0, true);
    //analogWrite(RPWM0, 0);
  } else if (currentMillis >= t3 && currentMillis < t4) {
    //setMotor(0, vel, false);
    //setMotor(1, vel1, false);
    setMotor(2, vel2, false);
    //analogWrite(RPWM0, 0);
  } else if (currentMillis >= t4) {
    setMotor(0, 0, false);
    setMotor(1, 0, false);
    setMotor(2, 0, false);
    //analogWrite(RPWM0, 0);
  }
}

//Función para dirección y velocidad del Motor
void setMotor(int motor, int vel, bool dir) {
  if (motor == 0) {
    if (vel > 0) {
      digitalWrite(EN0, HIGH);
    } else {
      digitalWrite(EN0, LOW);
    }
    if (dir) {
      analogWrite(LPWM0, 0);
      analogWrite(RPWM0, 0);
      analogWrite(RPWM0, vel);
    } else {
      analogWrite(LPWM0, 0);
      analogWrite(RPWM0, 0);
      analogWrite(LPWM0, vel);
    }
  } else if (motor == 1) {
    if (dir) {
      motorController.TurnRight(vel);
    } else {
      motorController.TurnLeft(vel);
    }
  } else if (motor == 2) {
    if (vel > 0) {
      digitalWrite(EN2, HIGH);
    } else {
      digitalWrite(EN2, LOW);
    }
    if (dir) {
      analogWrite(LPWM2, 0);
      analogWrite(RPWM2, 0);
      analogWrite(RPWM2, vel);
    } else {
      analogWrite(LPWM2, 0);
      analogWrite(RPWM2, 0);
      analogWrite(LPWM2, vel);
    }
  } else {
    Serial.print("mm a");
  }
}

//Función anti-rebote
bool debounce(byte input) {
  bool state = false;
  if (!digitalRead(input)) {
    delay(200);
    while (!digitalRead(input))
      ;
    delay(200);
    state = true;
  }
  return state;
}

//Función para la lectura del encoder
void leerEncoder0() {
  //Lectura de Velocidad
  if (modo)
    pulsos0++;  //Incrementa una revolución

  //Lectura de Posición
  else {
    int b = digitalRead(ENCODER_B0);
    if (b > 0) {
      //Incremento variable global
      theta0++;
    } else {
      //Decremento variable global
      theta0--;
    }
  }
}

void leerEncoder1() {
  //Lectura de Velocidad
  if (modo)
    pulsos1++;  //Incrementa una revolución

  //Lectura de Posición
  else {
    int b = digitalRead(ENCODER_B1);
    if (b > 0) {
      //Incremento variable global
      theta1++;
    } else {
      //Decremento variable global
      theta1--;
    }
  }
}

void leerEncoder2() {
  //Lectura de Velocidad
  if (modo)
    pulsos2++;  //Incrementa una revolución

  //Lectura de Posición
  else {
    int b = digitalRead(ENCODER_B2);
    if (b > 0) {
      //Incremento variable global
      theta2++;
    } else {
      //Decremento variable global
      theta2--;
    }
  }
}

void controlP() {
  unsigned long currentMillis = millis();
  //Ref2= 0;
  if (currentMillis >= 5000) {

    ATOMIC() {
      posicion1 = (float(-theta1 * 360.0 / resolution)) + 180;
      posicion2 = (float(theta2 * 360.0 / resolution)) - posicion1 + 15;
    }

    E1 = (float(RefQ1.number) - posicion1);
    E2 = (float(RefQ2.number) - posicion2);

    if (abs(E1) <= 1) {
      E1 = 0;
    }

    if (abs(E2) <= 0.1) {
      E2 = 0;
    }

    CmdP = kp1 * (E1);
    CmdP2 = kp2 * (E2);
  }
  if (currentMillis - previousMillis >= TsC) {
    previousMillis = currentMillis;

    float Cmd = CmdP + 260.0286 * ((g * m2 * l1 * cos((posicion1) * (PI / 180)) + g * m2 * lc2 * cos((posicion2 + posicion1) * (PI / 180)) + g * lc1 * m1 * cos((posicion1) * (PI / 180))));  //+ g*m2*l1*cos(posicion1)+g*m2*lc2*cos(posicion1+posicion2)+g*lc1*m1*cos(posicion1);
    float Cmd2 = CmdP2 + (g * lc2 * m2 * cos((posicion1 + posicion2) * (PI / 180))) * 260.0286;

    if (Cmd >= 0) {
      sentido1 = true;
    } else {
      sentido1 = false;
    }

    if (Cmd2 >= 0) {
      sentido2 = false;
    } else {
      sentido2 = true;
    }

    Cmd = abs(Cmd);
    Cmd2 = abs(Cmd2);

    float CmdLim = min(max(Cmd, 0), pwmMax);    // Saturated Control Output
    float CmdLim2 = min(max(Cmd2, 0), pwmMax);  // Saturated Control Output

    pwmDuty = int(CmdLim);    //int((CmdLim/1)*pwmMax);//int(CmdLim);//int((CmdLim/1)*pwmMax);
    pwmDuty2 = int(CmdLim2);  //int((CmdLim2/1)*pwmMax);

    if (currentMillis >= 5000) {
      setMotor(1, pwmDuty, sentido1);
      setMotor(2, pwmDuty2, sentido2);
    }
  }
}

float getFloat() {
  int cont = 0;
  FLOATUNION_t f;
  while (cont < 4) {  // ller 4 bytes (32bits por numero)
    f.bytes[cont] = Serial.read();
    cont = cont + 1;
  }
  return f.number;
}


void Envio() {
  q0M.number = float(posicion0);
  q1M.number = float(posicion1);
  q2M.number = float(posicion2);
  q3M.number = 0;
  Rq0.number = float(RefQ0.number);
  Rq1.number = float(RefQ1.number);
  Rq2.number = float(RefQ2.number);
  Rq3.number = float(RefQ3.number);

  // Print header: Important to avoid sync errors!
  if (Serial.availableForWrite() > 0) {
    Serial.write('A');
  }

  // Print float data
  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(q0M.bytes[i]);
    }
  }

  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(q1M.bytes[i]);
    }
  }

  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(q2M.bytes[i]);
    }
  }

  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(q3M.bytes[i]);
    }
  }

  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(Rq0.bytes[i]);
    }
  }

  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(Rq1.bytes[i]);
    }
  }

  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(Rq2.bytes[i]);
    }
  }

  for (int i = 0; i < 4; i++) {
    if (Serial.availableForWrite() > 0) {
      Serial.write(Rq3.bytes[i]);
    }
  }

  // Print terminator
  if (Serial.availableForWrite() > 0) {
    Serial.print('\n');
  }
}

void Recepcion() {
  if (Serial.available() > 1) {
    Serial.find('R');

    float aux = getFloat();
    if (!isnan(aux) && abs(aux - RefQ0.number) < 20) {
      RefQ0.number = aux;
    }

    aux = getFloat();
    if (!isnan(aux) && abs(aux - RefQ1.number) < 20) {
      RefQ1.number = aux;
    }

    aux = getFloat();
    if (!isnan(aux) && abs(aux - RefQ2.number) < 20) {
      RefQ2.number = aux;
    }

    aux = getFloat();
    if (!isnan(aux) && abs(aux - RefQ3.number) < 20) {
      RefQ3.number = aux;
    };


    Serial.find('\n');
  }
}
