#include <ESP32Servo.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Constantes PID
const double Kp = 3.0;
const double Ki = 0.0;
const double Kd = 100.0;

// Variables de control y estado
int pos = 0;
const int setPoint = 20;
unsigned long currentTime, previousTime = 0;
double elapsedTime;
double error, lastError = 0, cumError = 0, rateError;
double outPut;

// Variables para el sensor
float dis_cm;

// Definición del servo y sensor
Servo myservo;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  
  myservo.attach(2);
}

void loop() {
  dis_cm = medirDistancia();
  
  pos = PID(dis_cm) + 140;
  limitarPosicion();
  
  myservo.write(pos);
  
  Serial.print(setPoint);
  Serial.print(" ");
  Serial.println(dis_cm);
  
  delay(1000);
}

float medirDistancia() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter / 10.0;
  } else {
    Serial.println("Fuera de rango");
    return -1.0; // Indicador de error
  }
}

void limitarPosicion() {
  if (pos > 180) {
    pos = 180;
  } else if (pos < 20) {
    pos = 20;
  }
}

double PID(float input) {
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);

  error = setPoint - input;
  cumError += error * elapsedTime;
  rateError = (error - lastError) / elapsedTime;
  outPut = Kp * error + Ki * cumError + Kd * rateError;

  lastError = error;
  previousTime = currentTime;

  return outPut;
}
