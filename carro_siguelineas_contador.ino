#include <AFMotor.h>
#include <NewPing.h>

#define TRIGGER_PIN  A0  
#define ECHO_PIN     A5
#define MAX_DISTANCE 200 

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

// Definimos los sensores infrarrojos del siguelineas
//#define IR1 A0  // Sensor más a la izquierda
#define IR2 A1  // Sensor izquierda
#define IR3 A2  // Sensor central
#define IR4 A3  // Sensor derecha
#define IR5 A4  // Sensor más a la derecha

// Inicializamos los motores
AF_DCMotor motorL(2);  // Motor Izquierdo
AF_DCMotor motorD(1);  // Motor Derecho

// Variables para el controlador PID
float Kp = 5.0;  // Coeficiente Proporcional
float Ki = 0.0;  // Coeficiente Integral
float Kd = 3.0;  // Coeficiente Derivativo

float lastError = 0;
float integral = 0;

//Variables para el contador de casas
int contador = 0;
bool casa_detectada = false;


void setup() {
  Serial.begin(115200);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
}

void loop() {
  detectarCasa();
  if (contador == 2) {
    turnRight(150, 150);
    contador = 0;
  }
  // Leer los valores de los sensores
  int sensor2 = analogRead(IR2);
  int sensor3 = analogRead(IR3);
  int sensor4 = analogRead(IR4);

  // Calcular el error usando los valores de los sensores
  float error = calculateError(sensor2, sensor3, sensor4);

  // Obtener la corrección del controlador PID
  float correction = getPIDCorrection(error);

  // Aplicar la corrección a los motores
  applyMotorCorrection(correction);
  delay(50);  // Pequeña pausa para estabilizar las lecturas
}

void detectarCasa() {
  int distancia_casa = sonar.ping_cm();
  if (distancia_casa < 20 && !casa_detectada && distancia_casa != 0) {
    contador++;
    casa_detectada = true;
    Serial.println("Casa detectada");
  }
  if (distancia_casa > 20 && casa_detectada && distancia_casa != 0) {
    casa_detectada = false;
    Serial.println("Casa fuera de rango");
  }
}


float calculateError(int sensor2, int sensor3, int sensor4) {
  int threshold = 500;  // Ajuste el umbral para tu configuración específica
  float error = 0;
  //if (sensor1 < threshold) error += -2;  // Sensor más a la izquierda
  if (sensor2 < threshold) error += -1;  // Sensor izquierda
  if (sensor3 < threshold) error += 0;   // Sensor central
  if (sensor4 < threshold) error += 1;   // Sensor derecha
  return error;
}

float getPIDCorrection(float error) {
  float proportional = error;
  integral += error;
  float derivative = error - lastError;
  lastError = error;
  return Kp * proportional + Ki * integral + Kd * derivative;
}

void applyMotorCorrection(float correction) {
  int baseSpeed = 80;  // Velocidad base ajustada
  int maxSpeed = 100;  // Velocidad máxima permitida

  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  Serial.println(leftSpeed);
  Serial.println(rightSpeed);

  // Limitar las velocidades para estar dentro del rango permitido
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  // Aplicar la corrección según la dirección necesaria
  if (correction < 0) {
    turnLeft(abs(leftSpeed), abs(rightSpeed));
  } else if (correction > 0) {
    turnRight(abs(leftSpeed), abs(rightSpeed));
  } else {
    forward(leftSpeed, rightSpeed);
  }
}

// Funciones de movimiento
void forward(int leftSpeed, int rightSpeed) {
  motorL.run(FORWARD);
  motorL.setSpeed(leftSpeed);
  motorD.run(FORWARD);
  motorD.setSpeed(rightSpeed);
}

void turnLeft(int leftSpeed, int rightSpeed) {
  motorL.run(BACKWARD);
  motorL.setSpeed(leftSpeed);
  motorD.run(FORWARD);
  motorD.setSpeed(rightSpeed);
}

void turnRight(int leftSpeed, int rightSpeed) {
  motorL.run(FORWARD);
  motorL.setSpeed(leftSpeed);
  motorD.run(BACKWARD);
  motorD.setSpeed(rightSpeed);
}

void stop() {
  motorL.run(RELEASE);
  motorL.setSpeed(0);
  motorD.run(RELEASE);
  motorD.setSpeed(0);
}
