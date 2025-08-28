#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// Constantes del Servo.
#define SERVO_MIN_ANGLE 0 // Mínimo ángulo que barre el Servo.
#define SERVO_MAX_ANGLE 180 // Máximo ángulo que barre el Servo.
#define SERVO_REF_ANGLE 97 // Posición de referencia del Servo, nuestro cero.

// Constantes del Potenciómetro.
#define POT_MAX_ANALOG_VALUE 1023 // Se define el valor analógico máximo como 1023.
#define POT_MAX_ANGLE 300 // Se define el ángulo máximo del potenciómetro como 300°.

// Constantes temporales.
#define INTERVAL_SERVO 10000 // Se mueve el Servo cada INTERVAL_SERVO en ms
#define INTERVAL_SENSORS 20// Tasa de muestreo de los sensores: 20ms

// Objetos a usar.
Adafruit_MPU6050 mpu; // Se declara la IMU.
Servo my_servo; // Se declara el servo.

// Variables del Servo.
int servo_pin = 9; // Pin digital del servo.
int servo_min_angle_routine = 67; // ángulo mínimo de la rutina.
int servo_max_angle_routine = 127; // ángulo máximo de la rutina.
int servo_step_angle_routine = 60; // ángulo de movimiento en la rutina.
int servo_angle = servo_min_angle_routine; // Se setea la posición inicial de la rutina en -90° con respecto a la referencia. (0° reales)
bool sentido = false; 




// Variables temporales
unsigned long previousMillis_sensors = 0; // Se inicializa el tiempo previo como el tiempo inicial para la lectura de los sensores
unsigned long previousMillis_servo = 0; // Se inicializa el tiempo previo como el tiempo inicial para el Servo. 

// Variables de la IMU.
const float angleX_ref = -8.5; 
float angleX = 0; // Ángulo x de la IMU
const float alpha = 0.98; // Parámetro del filtro complementario.
float theta = 0 ; //Ángulo del péndulo procesado
float dt = 20e-3; // Se inicialzia el dt de muestreo de la IMU - POTENCIÓMETRO
float error_theta = 0;
float error_phi = 0;

// Variables del Potenciómetro.
const int pot_analog_pin = A0; // Se asigna el pin A0 para la lectura.
int pot_analog_value; // Variable que almacena la lectura analógica.
float phi = 125; // Variable que almacena el ángulo del potenciómetro (en °).
float pot_ref_angle = 169; // Angulo que coincide con la referencia del servo (los 97° del servo)

// Funciones de los sensores
void leer_sensores(float salidas[2], float alpha, float angleX_ref, float pot_ref_angle, int pot_analog_pin);

// Variables y funciones del controlador
float referencia [2] = {-30,0}; // Referencia:  phi | theta.
float u = 0.0;
float u_servo = 0.0;
float z[2] = {0.0, 0.0};  // error
float K_i[2] ={0.16, 0};  // Constante de integración. (solo aplicada a phi)
float K[4] = {200, 90, 12, 6}; // Ganancias de realimentación.
float M_ff [2] = {1.5, 0}; // Matriz de Feedforward (solo aplicada a phi)

// Variables y funciones del observador
const float A[4][4] = {{0.9769, -0.152, -0.001215, -0.02181}, {0.1523, 0.9838, -0.02593, 0.008431}, {-0.02706, -0.05485, 0.8, -0.0522}, {0.004625, 0.03261, 0.3389, 0.9926}};
const float B[4]    = {0.002765, -0.00137, 0.0122, -0.002472};
const float C[2][4] = {{-6.669, -4.565, -19.02, 6.812}, {-341.3, 53.95, -2.577, 4.176}};
const float L[4][2] = {{-0.0036, -0.0027}, {-0.0412, 0.0159}, {0.0391, -0.0049}, {0.3137, -0.0055}};  // Ganancia del observador

float x_hat[4] = {0.0, 0.0, 0.0, 0.0};  // Estimación de las variables de estado.
float x_hat_next[4] = {0.0, 0.0, 0.0, 0.0};
float y[2] = {0.0, 0.0};

void actualizar_observador(float u, float y[2], float x_hat[4], float x_hat_next[4]);






void setup() {
  // Setup del servo.
  my_servo.attach(servo_pin); // Se asocia el servo al pin.
  my_servo.write(97); // Se inicializa el servo en la posicion inicial utilizada para la rutina.

  // Setup de la IMU.
  Serial.begin(115200); // Se setean los baudios del puerto serial. 115200
  Wire.begin(); // Se inicializa la comuncación I2C.
  
  if (!mpu.begin()) { //Se inicializa la IMU.
    Serial.println("No se pudo encontrar el sensor MPU6050");
    while (1) delay(100);
  }
  Serial.println("MPU6050 listo!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Se usan 8G para la aceleración máxima de la IMU.
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // Se usan 500grad/s para la velocidad angular máxima que captura la IMU.
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ); //Se usan 44Hz como frecuencia de corte del filtro pasabajos.
  delay(5000); // Se espera para terminar de configurar todo.
}




void loop() {
  unsigned long currentMillis = millis();  // Se lee el tiempo actual.

  // ----------Rutina de lectura de los sensores
  if (currentMillis - previousMillis_sensors >= INTERVAL_SENSORS) //&& (pot_angle == SERVO_MIN_ANGLE_ROUTINE || pot_angle == SERVO_MAX_ANGLE_ROUTINE) ) { 
    {
      previousMillis_sensors = currentMillis; // Actualiza el tiempo anterior de lectura de los sensorescomo el tiempo actual.

     //---------------------------------------------------------------- INICIO SENSORES 
           // Procesamiento de los datos de la IMU.
      sensors_event_t a, g, t; // Se declaran las variables del sensor: a = acelerómetro, g= giroscopio, y t = termómetro (este último no se utiliza).
      mpu.getEvent(&a, &g, &t); // Se obtienen los datos del acelerómetro, del giroscopio y del termómetro.

      // Ángulos utilizando solamente las aceleraciones.
      // Se calcula el ángulo alrededor del eje X, usando los valores del acelerómetro.
      float accel_angleX = atan(a.acceleration.y/ a.acceleration.z);

      // Ángulos utilizando solamente las velocidades angulares.
      float gyro_angleX = angleX + g.gyro.x * dt; // Se calcula el ángulo alrededor de X usando la velocidad angular en torno a X.

      // Se aplica un filtro complementario para obtener los ángulos, eliminando el ruido y utilizando las medidas de los acelerómetros y giroscopios en simultáneo.
      // Utilizar la aceleración permite eliminar la deriva y la velocidad angular nos da suavidad sobre el ángulo.
      angleX = alpha * gyro_angleX + (1 - alpha) * accel_angleX; // Ángulo procesado alrededor de X.

      theta = angleX * RAD_TO_DEG + angleX_ref;

      // ----------Rutina de lectura del Potenciómetro


      // Se lee el valor analógico en el potenciómetro.
      pot_analog_value = analogRead(pot_analog_pin);

      // Se calcula el ángulo a partir de la lectura.
      phi =((pot_analog_value*1.0) / POT_MAX_ANALOG_VALUE) * POT_MAX_ANGLE;// Debe ser un float para que lo lea MATLAB.
      if(phi>=0)
      {
        phi=phi-pot_ref_angle;
      } else
        {
          phi=phi+pot_ref_angle; 
        }

    //------------------------------------------------------------------------------------------------------------------------- FIN SENSORES

      //---------------------------------------------------------------------------------Controlador (se calcula la señal de control)
      u = 0.0;
      y[0] = phi;
      y[1] = theta;

      // Se genera la acumulación del error, utilizada para la acción integral, aplicando anti-windup para evitar una acumulación excesiva que genere overflow.
      for (int i = 0; i < 2; i++) {
      z[i] += referencia[i] - y[i];
      z[i] = constrain(z[i], -20.0, 20.0);  // anti-windup
      }

      // Realimentación de estados
      for (int i = 0; i < 4; i++) {
      x_hat[i] = constrain(x_hat[i], -1000.0, 1000.0);  // evita que los estados estimados generen overflow, si se diera el caso
      u -= K[i] * x_hat[i];
      }

      // Acción integral
      for (int i = 0; i < 2; i++) {
      u += K_i[i] * z[i];
      }

      // Feedforward
      for (int i = 0; i < 2; i++) {
      u += M_ff[i] * referencia[i];
      }

      
      //---------------------------------------------------------------------------------Observador

      // Se calcula la salida estimada: y_hat = C_d * x_hat
      float y_hat[2] = {0.0, 0.0};
      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 4; j++) {
        y_hat[i] += C[i][j] * x_hat[j];
        }
      }

      // Calcula el error de la estimación a la salida: y - y_hat
      float error[2];
      for (int i = 0; i < 2; i++) {
      error[i] = y[i] - y_hat[i];
      }

      // Se calcula la siguiente estimación de los estados: x_hat_next = A_d * x_hat + B_d * u + L * error
     for (int i = 0; i < 4; i++) {
      x_hat_next[i] = 0.0;

      // Se calcula: A_d * x_hat
      for (int j = 0; j < 4; j++) {
        x_hat_next[i] += A[i][j] * x_hat[j];
      }

      // Se calcula: + B_d * u
      x_hat_next[i] += B[i] * u;

      // Se calcula: + L * error
     for (int j = 0; j < 2; j++) {
      x_hat_next[i] += L[i][j] * error[j];
      }
    }

      // Se actualiza la estimación de los estados.
      for (int i = 0; i < 4; i++) {
        x_hat[i] = x_hat_next[i];
      }

      // Se aplica la señal de control al servomotor, considerando tolerancias sobre las variables.
      error_theta = referencia[1]-theta;
      error_phi = referencia[0]-phi;
      if((abs(error_theta) > 2 ) || ( abs(g.gyro.x) > 0.3 )|| (abs(error_phi) > 5 ))
      {
      //u = constrain(u, -90, 90);
      u_servo = u + SERVO_REF_ANGLE;
      u_servo = constrain(u_servo, 0, 180);
      my_servo.write(u_servo);
      }

    // Se alterna la referencia phi_ref , para generar una rutina de movimiento.
    if (currentMillis - previousMillis_servo >= INTERVAL_SERVO) {
      previousMillis_servo = currentMillis;
      referencia[0] = sentido ? -30 : 30;  // En grados relativos (como phi)
      sentido = !sentido;
    }

    // Se envían los datos procesados a Matlab.
    matlab_send(phi,theta,g.gyro.x);
    }
}

// Función para enviar los datos a matlab.
void matlab_send(float x1, float x2, float x3) {
  Serial.write("abcd");
  byte * b = (byte *)&x1;
  Serial.write(b, 4);
  b = (byte *)&x2;
  Serial.write(b, 4);
  b = (byte *)&x3;
  Serial.write(b, 4);
}

