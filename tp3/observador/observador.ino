#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// Constantes del Servo.
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_REF_ANGLE 97

// Constantes del Potenciómetro.
#define POT_MAX_ANALOG_VALUE 1023
#define POT_MAX_ANGLE 300

// Constantes temporales.
#define INTERVAL_SERVO 10000
#define INTERVAL_SENSORS 20

// Objetos a usar.
Adafruit_MPU6050 mpu;
Servo my_servo;

// Variables del Servo.
int servo_pin = 9;
int servo_angle = SERVO_REF_ANGLE;
bool sentido = true;

// Variables temporales
unsigned long previousMillis_sensors = 0;
unsigned long previousMillis_servo = 0;

// Variables de la IMU.
const float angleX_ref = -8.5;
float angleX = 0;
const float alpha = 0.98;
float theta = 0;
float dt = 20e-3;

// Variables del Potenciómetro.
const int pot_analog_pin = A0;
int pot_analog_value;
float phi = 125;
float pot_ref_angle = 169;

// Variables y funciones del controlador
float u = 0.0;
float u_servo = 0.0;

// Variables del observador
const float A[4][4] = {{0.9769, -0.152, -0.001215, -0.02181}, {0.1523, 0.9838, -0.02593, 0.008431}, {-0.02706, -0.05485, 0.8, -0.0522}, {0.004625, 0.03261, 0.3389, 0.9926}};
const float B[4] = {0.002765, -0.00137, 0.0122, -0.002472};
const float C[2][4] = {{-6.669, -4.565, -19.02, 6.812}, {-341.3, 53.95, -2.577, 4.176}};
const float L[4][2] = {{-0.0036, -0.0027}, {-0.0412, 0.0159}, {0.0391, -0.0049}, {0.3137, -0.0055}};

float x_hat[4] = {0.0, 0.0, 0.0, 0.0};
float x_hat_next[4] = {0.0, 0.0, 0.0, 0.0};
float y[2] = {0.0, 0.0};

void setup() {
  my_servo.attach(servo_pin);
  my_servo.write(SERVO_REF_ANGLE);

  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("No se pudo encontrar el sensor MPU6050");
    while (1) delay(100);
  }
  Serial.println("MPU6050 listo!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  delay(5000);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis_sensors >= INTERVAL_SENSORS) {
    previousMillis_sensors = currentMillis;

    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    float accel_angleX = atan(a.acceleration.y / a.acceleration.z);
    float gyro_angleX = angleX + g.gyro.x * dt;
    angleX = alpha * gyro_angleX + (1 - alpha) * accel_angleX;
    theta = angleX * RAD_TO_DEG + angleX_ref;

    pot_analog_value = analogRead(pot_analog_pin);
    phi = ((pot_analog_value * 1.0) / POT_MAX_ANALOG_VALUE) * POT_MAX_ANGLE - pot_ref_angle;

    y[0] = phi;
    y[1] = theta;

    float y_hat[2] = {0.0, 0.0};
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 4; j++) {
        y_hat[i] += C[i][j] * x_hat[j];
      }
    }

    float error[2];
    for (int i = 0; i < 2; i++) {
      error[i] = y[i] - y_hat[i];
    }

    float u_observer = u_servo - SERVO_REF_ANGLE;

    for (int i = 0; i < 4; i++) {
      x_hat_next[i] = 0.0;
      for (int j = 0; j < 4; j++) {
        x_hat_next[i] += A[i][j] * x_hat[j];
      }
      x_hat_next[i] += B[i] * u_observer;
      for (int j = 0; j < 2; j++) {
        x_hat_next[i] += L[i][j] * error[j];
      }
    }

    for (int i = 0; i < 4; i++) {
      x_hat[i] = x_hat_next[i];
    }

    if (currentMillis - previousMillis_servo >= INTERVAL_SERVO) {
      previousMillis_servo = currentMillis;

      int offset = sentido ? 20 : -20;
      int target_angle = SERVO_REF_ANGLE + offset;
      target_angle = constrain(target_angle, 0, 180);

      u_servo = target_angle;
      my_servo.write(u_servo);
      sentido = !sentido;
    }

    matlab_send(phi, theta, g.gyro.x,10,10);
  }
}

void matlab_send(float d1, float d2, float d3, float d4, float d5) {
  Serial.write("abcd");
  byte * b = (byte *)&d1;
  Serial.write(b, 4);
  b = (byte *)&d2;
  Serial.write(b, 4);
  b = (byte *)&d3;
  Serial.write(b, 4);
  b = (byte *)&d4;
  Serial.write(b, 4);
    b = (byte *)&d5;
  Serial.write(b, 4);
}
