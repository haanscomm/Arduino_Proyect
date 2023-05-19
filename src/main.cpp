#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Definimos variables booleanas para distintas funciones
boolean estado = true;
boolean pulsar = false;

/***  Variables  ***/
#define SERVOMIN 320  // Valor de la longitud mínima del pulso PWM para velocidad máxima en sentido antihorario (valor de 0 a 4096).
#define SERVOSTOP 380 // Valor de la longitud del pulso para detener el servo (valor de 0 a 4096).
#define SERVOMAX 440  // Valor de la longitud máxima del pulso PWM para velocidad máxima en sentido horario (valor de 0 a 4096).
#define servo_left 0  // Servo izquierdo conectado al canal 0 del PWM shield.
#define servo_right 1 // Servo derecho conectado al canal 1 del PWM shield.
#define pingPin 5     // Definimos el pin del sensor ultrasonido (sensor que detecta si hay algún objeto delante del robot)
#define green_led 9   // Definimos el pin del led verde
#define red_led 12    // Definimos el pin del led rojo
#define button_pin 11 // Definimos el pin del pulsador
#define Buzzpin 13    // Definimos el pin correspondiente al zumbador
// Definimos variables para el sonido junto la frecuencia de cada nota musical
#define DO 523
#define RE 587
#define MI 659
#define IR_left 2                // Sensor IR izquierdo conectado al pin digital 2
#define IR_right 3               // Sensor IR derecho conectado al pin digital 3
#define longitud_giro 400        // Definimos el parámetro para el giro en la espiral (debe ser un valor más cercana a SERVOMAX dado que la espiral gira en sentido antihorario)
#define longitudinversa_giro 330 // Definimos el parámetro para el giro en la espiral inversa (debe ser un valor cercano a SERVOMIN dado que la espiral girará en sentido horario)
#define espera 1000              // Definimos el valor para el tiempo de espera
#define SERVO_90deg 420          // Servo a 90º (sensor de ultrasonido mirando hace delante).

// #define maxParpadeo 4

/***   Variables Globales   ***/
const int servo_180 = 2; // Servo conectado al canal 2 del PWM shield.

// Método para que el robot realice la espiral
void espiral(int i, int valor)
{
  digitalWrite(green_led, HIGH); // LED verde encendido
  digitalWrite(red_led, LOW);    // LED rojo apagado
  pwm.setPWM(servo_left, 0, longitud_giro - i);
  pwm.setPWM(servo_right, 0, SERVOMAX);
  delay(espera * valor);
}

void espiral_inversa(int i, int valor)
{
  digitalWrite(red_led, HIGH); // LED rojo encendido

  pwm.setPWM(servo_left, 0, SERVOMIN);
  pwm.setPWM(servo_right, 0, longitudinversa_giro + i);
  delay(espera * valor);
}

// Función para Calcular la distancia
long CalcularDistancia()
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

// Método que detecta un obstáculo
void DetectarObstaculo()
{
  // Guardamos el valor de la distancia en una variable para compararla después.
  long distancia = CalcularDistancia();
  // Debe apagar el LED que tuviera encendido antes de detectar el obstáculo. (verde)
  digitalWrite(green_led, LOW);
  // Debe encender el LED que tuviera apagado antes de detectar el obstáculo. (rojo)
  digitalWrite(red_led, HIGH);

  if (distancia <= 14)
  {
    // Paramos el robot cuatro segundos
    pwm.setPWM(servo_left, 0, SERVOSTOP);
    pwm.setPWM(servo_right, 0, SERVOSTOP);
    delay(espera * 4);

    // El robot gira 180º
    pwm.setPWM(servo_left, 0, SERVOMIN);
    pwm.setPWM(servo_right, 0, SERVOSTOP);
    delay(espera * 3);

    // Debe realizar una espiral inversa
    espiral_inversa(20, 2);
    espiral_inversa(40, 4);
    espiral_inversa(60, 6);
    espiral_inversa(80, 8);
  }
}

// Método para que el robot haga sonidos cuando acabe la espiral en el método Pulsador
void Sonido()
{
  /*digitalWrite(green_led, LOW);
  digitalWrite(red_led, LOW);*/
  tone(Buzzpin, DO); // Mediante la función tone() indicamos el pin de salida y la frecuencia de la señal cuadrada
  delay(espera * 3);
  tone(Buzzpin, RE); // Repetimos para cada nota creando la escala musical
  delay(espera * 3);
  tone(Buzzpin, MI);
  delay(espera * 3);
  tone(Buzzpin, DO);
  delay(espera * 3);
  tone(Buzzpin, RE);
  delay(espera * 3);
  tone(Buzzpin, MI);
  delay(espera * 3);
  noTone(Buzzpin);     // Detenemos la generación de la señal cuadrada
  delay(espera * 0.3); // durante 300 ms
}

// Método que cambia la asignación del booleano para que la interrupción funcione
void on_Pulsador()
{
  pulsar = !pulsar;
}

// Método Pulsador
void Pulsador()
{

  pinMode(button_pin, INPUT);
  int button_value = digitalRead(button_pin);

  // Cuando el botón está presionado, se de tiene la marcha durante 4 segundos
  digitalWrite(green_led, LOW);
  digitalWrite(red_led, LOW);
  estado = false;

  // Se para cuatro segundos
  pwm.setPWM(servo_left, 0, SERVOSTOP);
  pwm.setPWM(servo_right, 0, SERVOSTOP);
  delay(espera * 4);

  // Espiral hacia el centro
  espiral(50, 5);
  espiral(40, 5);
  espiral(30, 4);
  espiral(20, 3);
  espiral(10, 2);

  // Se para durante un segundo y después emite un sonido
  pwm.setPWM(servo_left, 0, SERVOSTOP);
  pwm.setPWM(servo_right, 0, SERVOSTOP);
  delay(espera);
  Sonido();
}

// Nétodo del parapadeo de las luces
void Parpadeo()
{
  if (red_led == HIGH)
  {
    digitalWrite(red_led, HIGH);
    delay(espera);
    digitalWrite(red_led, LOW);
    delay(espera);
    digitalWrite(red_led, HIGH);
    delay(espera);
    digitalWrite(red_led, LOW);
    delay(espera);

  }
  else if (green_led == HIGH)
  {
    digitalWrite(green_led, HIGH);
    delay(espera);
    digitalWrite(green_led, LOW);
    delay(espera);
    digitalWrite(green_led, HIGH);
    delay(espera);
    digitalWrite(green_led, LOW);
    delay(espera);

  }
}

void DetectarMancha() // cero = negro, uno = blanco
{
  pinMode(IR_left, INPUT);  // Configuramos el pin 2 donde se conectan los sensor IR izquierdo como INPUT
  pinMode(IR_right, INPUT); // Configuramos el pin 3 donde se conectan los sensor IR izquierdo como INPUT

  int valor_IR_left = digitalRead(IR_left);   // Leemos la entrada digital 0 donde está conectado el sensor IR izquierdo
  int valor_IR_right = digitalRead(IR_right); // Leemos la entrada digital 1 donde está conectado el sensor IR derecho

  if (valor_IR_left == 0 || valor_IR_right == 0)
  {
    pwm.setPWM(servo_left, 0, SERVOSTOP);
    pwm.setPWM(servo_right, 0, SERVOSTOP);
    delay(espera * 4);

    tone(Buzzpin, MI);
    delay(espera * 2);
    tone(Buzzpin, DO);
    delay(espera * 2);
    noTone(Buzzpin);

    Parpadeo();
  }
}

void DetectarLuz()
{

  int light = analogRead(A0); // Leemos la entrada analógica 0 donde está conectado el sensor de luz izquierdo

  if (light <= 51)
  {

    // El robot para
    pwm.setPWM(servo_left, 0, SERVOSTOP);
    pwm.setPWM(servo_right, 0, SERVOSTOP);
    delay(espera * 2);

    // Encendemos los dos leds
    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, HIGH);
    delay(espera * 3);

    // Espiral a velocidad reducida
    espiral(40, 4);
    espiral(50, 5);
    espiral(60, 6);
  }
}

void setup()
{
  // initialize serial communication:
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);

  // Interupción para el pulsador
  attachInterrupt(digitalPinToInterrupt(button_pin), on_Pulsador, RISING);
  
}

void loop()
{

  while (estado)
  {

    //  primera vuelta
    espiral(0, 1);
    DetectarObstaculo();
    DetectarMancha();
    DetectarLuz();

    // segunda vuelta
    espiral(10, 2);
    DetectarObstaculo();
    DetectarMancha();
    DetectarLuz();

    // tercera vuelta
    espiral(20, 3);
    DetectarLuz();
    DetectarObstaculo();
    DetectarMancha();

    if (pulsar == true)
    {
      Pulsador();
    }

    // cuarta vuelta
    DetectarLuz();
    espiral(30, 4);
    DetectarObstaculo();
    DetectarMancha();

    // quinta vuelta
    DetectarLuz();
    espiral(40, 5);
    DetectarObstaculo();
    DetectarMancha();

    // sexta vuelta
    DetectarLuz();
    espiral(50, 6);
    DetectarObstaculo();
    DetectarMancha();

    if (pulsar == true)
    {
      Pulsador();
    }

    // septima vuelta
    DetectarLuz();
    espiral(60, 7);
    DetectarObstaculo();
    DetectarMancha();

    // octava vuelta
    DetectarLuz();
    espiral(70, 8);
    DetectarObstaculo();
    DetectarMancha();

    // última vuelta
    DetectarLuz();
    espiral(80, 9);
    DetectarObstaculo();
    DetectarMancha();
  }
}