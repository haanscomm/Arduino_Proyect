#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

boolean estado = true;
boolean pulsar = false;
boolean mancha = false;
// boolean tiempo = false;
#define pingPin 5
#define SERVOMIN 320  // Valor de la longitud mínima del pulso PWM para velocidad máxima en sentido antihorario (valor de 0 a 4096).
#define SERVOSTOP 380 // Valor de la longitud del pulso para dentener el servo (valor de 0 a 4096).
#define SERVOMAX 440
#define servo_left 0  // Servo izquierdo conectado al canal 0 del PWM shield.
#define servo_right 1 // Servo derecho conectado al canal 1 del PWM shield.
#define green_led 9
#define red_led 12
#define longitud_giro 400 // Parametro para el giro en espiral
#define longitudinversa_giro 330
#define espera 1000     // Parametro para el tiempo de espera
#define SERVO_90deg 420 // Servo a 90º (sensor de ultrasonido mirando hace delante).
#define button_pin 11
// definimos variables para el sonido
#define DO 523 // Definimos la frecuencia de cada nota musical
#define RE 587
#define MI 659

#define Buzzpin 13 // Definimos el pin al que conectamos el zumbador
#define IR_left 2  // Sensor IR izquierdo conectado al pin digital 2
#define IR_right 3 // Sensor IR derecho conectado al pin digital 3

#define maxParpadeo 4

/***   Variables Globales   ***/
const int servo_180 = 2; // Servo conectado al canal 2 del PWM shield.

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
    pwm.setPWM(servo_left, 0, SERVOMIN);
    pwm.setPWM(servo_right, 0, SERVOMAX);
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
    pwm.setPWM(servo_left, 0, SERVOMIN);
    pwm.setPWM(servo_right, 0, SERVOMAX);
  }
}

// Método para que el robot realice la espiral
void espiral(int i, int valor)
{
  digitalWrite(green_led, HIGH); // LED verde encendido
  digitalWrite(red_led, LOW); // LED rojo apagado
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

// Método para que el robot haga sonidos cuando acabe la espiral en el método Pulsador
void Sonido()
{
  digitalWrite(green_led, LOW);
  digitalWrite(red_led, LOW);
  tone(Buzzpin, DO); // Mediante la función tone() indicamos el pin de salida y la frecuencia de la señal cuadrada
  delay(espera * 3);
  // delay(300); // Retraso de 300 ms para fijar la duración de la nota
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

/*void on_Mancha(){
  mancha = !mancha;
}*/

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
    delay(espera * 3);
    tone(Buzzpin, DO);
    delay(espera * 3);
    tone(Buzzpin, RE);
    noTone(Buzzpin);  
    Parpadeo();
    pwm.setPWM(servo_left, 0, SERVOMIN);
    pwm.setPWM(servo_right, 0, SERVOMAX);
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

    // tercera vuelta
    espiral(40, 4);

    // cuarta vuelta
    espiral(50, 5);

    // quinta vuelta
    espiral(60, 6);
  }

  delay(200); // Retraso entre lecturas de 200 ms
}

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
    delay(espera * 4);

    // INTENTO QUE EL SERVO SE GURE 180 GRADOS
    // Posición 90º, lo hacemos dos veces porque se tiene que girar 180 grados

    pwm.setPWM(servo_left, 0, SERVOMIN);
    pwm.setPWM(servo_right, 0, SERVOSTOP);
    delay(espera * 3);

    espiral_inversa(20, 2);
    espiral_inversa(40, 4);
    DetectarMancha();
    espiral_inversa(60, 6);
    espiral_inversa(80, 8);
    DetectarMancha();
        // espiral_inversa(100, 10);
  }
}

// Método para que el robot se pare 4 segundos cuando pulsamos el botón, que describa una espiral hasta el centro y se detenga
void on_Pulsador()
{
  pulsar = !pulsar;
}


void Pulsador(){
    pinMode(button_pin, INPUT);
   int button_value = digitalRead(button_pin);

  // Cuando el botón está presionado, se de tiene la marcha durante 4 segundos
    digitalWrite(green_led, LOW);
    digitalWrite(red_led, LOW);
    estado = false;

    // Serial.println("Pressed");
    pwm.setPWM(servo_left, 0, SERVOSTOP);
    pwm.setPWM(servo_right, 0, SERVOSTOP);
    delay(espera * 4);

    espiral(50, 5);

    espiral(40, 5);

    espiral(30, 4);

    espiral(20, 3);

    espiral(10, 2);

    pwm.setPWM(servo_left, 0, SERVOSTOP);
    pwm.setPWM(servo_right, 0, SERVOSTOP);
    delay(espera);
    Sonido();
 
}

void setup()
{
  // initialize serial communication:
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  // pinMode(button_pin, INPUT);

  // Interupción para el pulsador
  attachInterrupt(digitalPinToInterrupt(button_pin), on_Pulsador, RISING);
  // Interrupción del sensor izq
  attachInterrupt(digitalPinToInterrupt(IR_left), DetectarMancha, RISING);
  // Interrupción del sensor drcho
  attachInterrupt(digitalPinToInterrupt(IR_right), DetectarMancha, RISING);
}

void loop()
{
    
  while (estado)
  {

    //  primera vuelta
    DetectarLuz();
    espiral(0, 1);
    calcularDistancia();
    DetectarMancha();
    // Pulsador();

    // segunda vuelta
    espiral(10, 2);
    calcularDistancia();
    DetectarLuz();
    // Pulsador();
    DetectarMancha();

    /*if(mancha == true)
{
  DetectarMancha();
}    */

// Pulsador();

    // tercera vuelta
    DetectarLuz();
    espiral(20, 3);
    calcularDistancia();

    if(pulsar == true){
      Pulsador();
    }

   DetectarMancha();

    // cuarta vuelta
    DetectarLuz();
    espiral(30, 4);
    calcularDistancia();
    // Pulsador();
   DetectarMancha();

    // quinta vuelta
    DetectarLuz();
    espiral(40, 5);
    calcularDistancia();
    // Pulsador();
     DetectarMancha();

    // sexta vuelta
    DetectarLuz();
    espiral(50, 6);
    calcularDistancia();
    // Pulsador();
     DetectarMancha();

    if(pulsar == true){
      Pulsador();
    }

    // septima vuelta
    DetectarLuz();
    espiral(60, 7);
    calcularDistancia();
    // Pulsador();
    DetectarMancha();

    // octava vuelta
    DetectarLuz();
    espiral(70, 8);
    calcularDistancia();
    // Pulsador();
    DetectarMancha();

    // última vuelta
    DetectarLuz();
    espiral(80, 9);
    calcularDistancia();
    // Pulsador();
    DetectarMancha();

  
}
}