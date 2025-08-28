#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// Constantes del Servo.
#define SERVO_MIN_ANGLE_ROUTINE 52 //  Ángulo mínimo de la rutina del Servo.(97-45)
#define SERVO_MAX_ANGLE_ROUTINE 142 //  Ángulo máximo de la rutina del Servo. (97 + 45)
#define SERVO_STEP_ANGLE_ROUTINE 90 // Ángulo que barre el Servo en cada paso de la rutina.
#define SERVO_MIN_ANGLE 0 // Mínimo ángulo que barre el Servo.
#define SERVO_MAX_ANGLE 180 // Máximo ángulo que barre el Servo.
#define SERVO_REF_ANGLE 97 // Posición de referencia del Servo, nuestro cero.

// Constantes del Potenciómetro.
#define POT_MAX_ANALOG_VALUE 1023 // Se define el valor analógico máximo como 1023.
#define POT_MAX_ANGLE 300 // Se define el ángulo máximo del potenciómetro como 300°.

// Constantes temporales.
#define INTERVAL_SERVO 10000 // Se mueve el Servo cada 5000ms
#define INTERVAL_SENSORS 10 // Tasa de muestreo de los sensores: 10ms

// Objetos a usar.
Adafruit_MPU6050 mpu; // Se declara la IMU.
Servo my_servo; // Se declara el servo.

// Variables del Servo.
int servo_pin = 9; // Pin digital del servo.
int servo_angle = SERVO_MIN_ANGLE_ROUTINE; // Se setea la posición inicial de la rutina en -90° con respecto a la referencia. (0° reales)

// Variables temporales
unsigned long previousMillis_sensors = 0; // Se inicializa el tiempo previo como el tiempo inicial para la lectura de los sensores
unsigned long previousMillis_servo = 0; // Se inicializa el tiempo previo como el tiempo inicial para el Servo. 

// Variables de la IMU.
const float angleX_ref = -8.5; // -173.5;
float angleX = 0; //, angleY = 0, angleZ = 0; // Ángulos procesados tras la lectura de la IMU.
const float alpha = 0.98; // Parámetro del filtro complementario.
float angleX_matlab = angleX_ref ;
float dt = 20e-3; // Se inicialzia el dt de muestreo de la IMU - POTENCIÓMETRO

// Variables del Potenciómetro.
const int pot_analog_pin = A0; // Se asigna el pin A0 para la lectura.
int pot_analog_value; // Variable que almacena la lectura analógica.
float pot_angle = 125; // Variable que almacena el ángulo del potenciómetro (en °).
float pot_ref_angle = 169; // Angulo que coincide con la referencia del servo (los 97° del servo)


void setup() {
  // Setup del servo.
  my_servo.attach(servo_pin); // Se asocia el servo al pin.
  my_servo.write(servo_angle); // Se inicializa el servo en la posicion inicial utilizada para la rutina.

  // Setup de la IMU.
  Serial.begin(115200); // Se setean los baudios del puerto serial.
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

    // ----------Rutina de lectura de la IMU

    // Procesamiento de los datos de la IMU.
    sensors_event_t a, g, t; // Se declaran las variables del sensor: a = acelerómetro, g= giroscopio, y t = termómetro (este último no se utiliza).
    mpu.getEvent(&a, &g, &t); // Se obtienen los datos del acelerómetro, del giroscopio y del termómetro.

    // Ángulos utilizando solamente las aceleraciones.
    // Se calcula el ángulo alrededor del eje X, usando los valores del acelerómetro.
    float accel_angleX = atan(a.acceleration.y/ a.acceleration.z);
    // Se calcula el ángulo alrededor del eje Y, usando los datos del acelerómetro.
    //float accel_angleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));

    // Ángulos utilizando solamente las velocidades angulares.
    float gyro_angleX = angleX + g.gyro.x * dt; // Se calcula el ángulo alrededor de X usando la velocidad angular en torno a X.
    //float gyro_angleY = angleY + g.gyro.y * dt; // Se calcula el ángulo alrededor de Y usando la velocidad angular en torno a Y.
    //float gyro_angleZ = angleZ + g.gyro.z * dt; // Se calcula el ángulo alrededor de Z usando la velocidad angular en torno a Z.

    // Se aplica un filtro complementario para obtener los ángulos, eliminando el ruido y utilizando las medidas de los acelerómetros y giroscopios en simultáneo.
    // Utilizar la aceleración permite eliminar la deriva y la velocidad angular nos da suavidad sobre el ángulo.
    angleX = alpha * gyro_angleX + (1 - alpha) * accel_angleX; // Ángulo procesado alrededor de X.
    //angleY = alpha * gyro_angleY +  (1 - alpha) * accel_angleY; // Ángulo procesado alrededor de Y.
    //angleZ = gyro_angleZ; // Ángulo procesado alrededor de Z.

    angleX_matlab = angleX * RAD_TO_DEG + angleX_ref;

    // ----------Rutina de lectura del Potenciómetro

    // Se lee el valor analógico en el potenciómetro.
    pot_analog_value = analogRead(pot_analog_pin);

    // Se calcula el ángulo a partir de la lectura.
    pot_angle =((pot_analog_value*1.0) / POT_MAX_ANALOG_VALUE) * POT_MAX_ANGLE; // Debe ser un float para que lo lea MATLAB.

    // Se envían los datos procesados a Matlab.
    matlab_send(pot_angle-pot_ref_angle, angleX_matlab);
  }



  // ----------Rutina de movimiento del Servo - Debe ir el controlador próximamente

  // Mueve el Servo en intervalos de 500ms.
  if (currentMillis - previousMillis_servo >= INTERVAL_SERVO) {
    previousMillis_servo = currentMillis; //Actualiza el tiempo anterior del movimiento del servo como el tiempo actual.

    // Se actualiza la posición del servo.
    servo_angle += SERVO_STEP_ANGLE_ROUTINE;

    // Reinicia la posición del servo si alcanza 180°.
    if (servo_angle > SERVO_MAX_ANGLE_ROUTINE) {
      servo_angle = SERVO_MIN_ANGLE_ROUTINE; 
    }

    // Se mueve el servo a la nueva posición.
    my_servo.write(servo_angle);  
  }
}

// Función para enviar los datos a matlab.
void matlab_send(float ax, float ay) {
  Serial.write("abcd");
  byte * b = (byte *)&ax;
  Serial.write(b, 4);
  b = (byte *)&ay;
  Serial.write(b, 4);
}