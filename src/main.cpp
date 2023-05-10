#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define pingPin 5

boolean estado = true;
// boolean tiempo = false;
#define SERVOMIN 320  // Valor de la longitud mínima del pulso PWM para velocidad máxima en sentido antihorario (valor de 0 a 4096).
#define SERVOSTOP 380 // Valor de la longitud del pulso para dentener el servo (valor de 0 a 4096).
#define SERVOMAX 440
#define servo_left 0  // Servo izquierdo conectado al canal 0 del PWM shield.
#define servo_right 1 // Servo derecho conectado al canal 1 del PWM shield.
#define green_led 9
#define red_led 5
#define longitud_giro 400 // Parametro para el giro en espiral
#define espera 2000       // Parametro para el tiempo de espera
#define SERVO_90deg 420   // Servo a 90º (sensor de ultrasonido mirando hace delante).
#define button_pin 11

/***   Variables Globales   ***/
const int servo_180 = 2; // Servo conectado al canal 2 del PWM shield.

void setup()
{
  // initialize serial communication:
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
}

// Funcion ultrasonido
long funcion_ultrasonido()
{
  long duration;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(10);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  return duration / 29 / 2;
}

// Método para que el robot realice la espiral
void espiral(int i, int valor)
{
  digitalWrite(green_led, HIGH); // LED verde encendido
  pwm.setPWM(servo_left, 0, longitud_giro - i);
  pwm.setPWM(servo_right, 0, SERVOMAX);
  delay(espera * valor);
}

// Método para que el robot se pare cada vez que detecte un obstáculo
void calcularDistancia()
{
  long distancia = funcion_ultrasonido();
  // Debe apagar el LED que tuviera encendido antes de detectar el obstáculo.
  digitalWrite(green_led, LOW);
  // Debe encender el LED que tuviera apagado antes de detectar el obstáculo.
  digitalWrite(red_led, HIGH);
  if (distancia <= 14)
  {
    pwm.setPWM(servo_left, 0, SERVOSTOP);
    pwm.setPWM(servo_right, 0, SERVOSTOP);
    delay(espera * 2);
    // INTENTO QUE EL SERVO SE GURE 180 GRADOS
    // Posición 90º, lo hacemos dos veces porque se tiene que girar 180 grados
    pwm.setPWM(servo_180, 0, SERVO_90deg);
    delay(2000);

    pwm.setPWM(servo_180, 0, SERVO_90deg);
    delay(2000);
  }
}

// Método para que el robot se pare 4 segundos cuando pulsamos el botón, que describa una espiral hasta el centro y se detenga
void Pulsador()
{
  pinMode(button_pin, INPUT);
  int button_value = digitalRead(button_pin);

  // Cuando el botón está presionado, se de tiene la marcha durante 4 segundos
  do
  {
    // Serial.println("Pressed");
    pwm.setPWM(servo_left, 0, SERVOSTOP);
    pwm.setPWM(servo_right, 0, SERVOSTOP);
    delay(espera * 2);

    // Primera vuelta
    espiral(200, 150);
    calcularDistancia();
    Pulsador();
    // Segunda vuelta
    espiral(0, 0);
    calcularDistancia();
    Pulsador();

    // Tercera vuelta
    espiral(50, 15);
    calcularDistancia();
    Pulsador();

    // Cuarta vuelta
    espiral(100, 50);
    calcularDistancia();
    Pulsador();

    // Ultima vuelta
    espiral(150, 100);
    calcularDistancia();
    Pulsador();

  } while (button_value == HIGH);
}



void loop()
{

  // primera vuelta
  espiral(0, 0);
  calcularDistancia();
  Pulsador();

  // segunda vuelta
  espiral(50, 15);
  calcularDistancia();
  Pulsador();

  // tercera vuelta
  espiral(100, 50);
  calcularDistancia();
  Pulsador();

  // cuarta vuelta
  espiral(150, 100);
  calcularDistancia();
  Pulsador();

  // ultima vuelta
  espiral(200, 150);
  calcularDistancia();
  Pulsador();

  // FORMA SIN PARAMETRIZAR
  /*
    // gira durante 1 segundo con la rueda izquierda a 400
    digitalWrite(green_led, HIGH); // LED verde encendido
    pwm.setPWM(servo_left, 0, longitud_giro);
    pwm.setPWM(servo_right, 0, SERVOMAX);
    delay(espera);
    calcularDistancia();

    // gira durante 2 segundos y la rueda izquierda con 390 de velocidad
    // digitalWrite(green_led, HIGH);
    pwm.setPWM(servo_left, 0, longitud_giro-50);
    pwm.setPWM(servo_right, 0, SERVOMAX);
    delay(espera*15);
    calcularDistancia();

    // gira durante 3 segundos y la rueda izquierda con 380 de velocidad
    // digitalWrite(green_led, HIGH); // LED verde encendido
    pwm.setPWM(servo_left, 0, longitud_giro-100);
    pwm.setPWM(servo_right, 0, SERVOMAX);
    delay(espera*50);//100000
    calcularDistancia();

    // gira durante 4 segundos y la rueda izquierda con 370 de velocidad
    // digitalWrite(green_led, HIGH); // LED verde encendido
    pwm.setPWM(servo_left, 0, longitud_giro-150);
    pwm.setPWM(servo_right, 0, SERVOMAX);
    delay(espera*100);
    calcularDistancia();

    // gira durante 5 segundos y la rueda izquierda con 360 de velocidad
    // digitalWrite(green_led, HIGH); // LED verde encendido
    pwm.setPWM(servo_left, 0, longitud_giro-200);
    pwm.setPWM(servo_right, 0, SERVOMAX);
    delay(espera*150);
    //delay(espera*400) 800000 o yo pondria mejor espera*150 = 300.000
    calcularDistancia();


    */
  /*
        // gira durante 6 segundos y la rueda izquierda con 350 de velocidad
        // digitalWrite(green_led, HIGH); // LED verde encendido
        pwm.setPWM(servo_left, 0, 350);
        pwm.setPWM(servo_right, 0, SERVOMAX);
        delay(6000);

        // gira durante 7 segundos y la rueda izquierda con 340 de velocidad
        // digitalWrite(green_led, HIGH); // LED verde encendido
        pwm.setPWM(servo_left, 0, 340);
        pwm.setPWM(servo_right, 0, SERVOMAX);
        delay(7000);

        // gira durante 8 segundos y la rueda izquierda con 330 de velocidad
        // digitalWrite(green_led, HIGH); // LED verde encendido
        pwm.setPWM(servo_left, 0, 330);
        pwm.setPWM(servo_right, 0, SERVOMAX);
        delay(8000);

        // gira durante 9 segundos y la rueda izquierda con 320 de velocidad
        // digitalWrite(green_led, HIGH); // LED verde encendido
        pwm.setPWM(servo_left, 0, 320);
        pwm.setPWM(servo_right, 0, SERVOMAX);
        delay(9000);
      */
}
/*long distancia = funcion_ultrasonido();
// if (estado)
{
  pwm.setPWM(servo_left, 0, SERVOMIN);
  pwm.setPWM(servo_right, 0, SERVOMAX);
  // }

  Serial.println(distancia);

  if (distancia <= 14)
  {
    pwm.setPWM(servo_left, 0, SERVOSTOP);
    pwm.setPWM(servo_right, 0, SERVOSTOP);
    delay(4000);
  }*/

// long distancia = funcion_ultrasonido();

/* if (distancia < 14)
 {
   pwm.setPWM(servo_left, 0, SERVOSTOP);
   pwm.setPWM(servo_right, 0, SERVOSTOP);
   delay(4000);
 }
 else
 {*/
