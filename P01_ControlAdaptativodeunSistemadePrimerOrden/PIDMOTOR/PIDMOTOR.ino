#include <SingleEMAFilterLib.h>
SingleEMAFilter<float> singleEMAFilter(0.4);
// Definición de pines
const int pinEncoder = 2;   // Pin para leer el encoder
const int pinMotorP = 6;     // Pin para controlar el motor
const int pinMotorN = 5;     // Pin para controlar el motor
float o1 = 0.0;

// Variables PID
volatile long encoderPos = 0;  // Posición del encoder
float setpoint = 80.0;          // Posición deseada
float inputf = 0.0;
float input = 0.0;             // Posición actual
float output = 0.0;            // Salida del PID
float Kp = 6.0, Ki = 1.5, Kd = 0.1;  // Ganancias PID
float rpm = 0;
// Variables para el cálculo PID
float lastInput = 0.0;
float integral = 0.0;

// Tiempo de muestreo
const float sampleTime = 0.1;  // 100 ms

void leerSetpoint() {
  if (Serial.available() > 0) {
    // Leer la cadena completa desde el Serial
    String inputString = Serial.readStringUntil('\n');
    
    // Verificar que la cadena no esté vacía
    if (inputString.length() > 0) {
      // Convertir la cadena a float
      float newSetpoint = inputString.toFloat();
      
      // Verificar que el valor es válido antes de asignarlo
      if (newSetpoint != 0.0 || inputString == "0") {
        setpoint = newSetpoint;
        Serial.print("Nuevo setpoint: ");
        Serial.println(setpoint);
      } else {
        Serial.println("Entrada no válida");
      }
    }
  }
}


// Interrupción por temporizador (Timer1)
void setupTimer() {
  cli(); // Desactiva las interrupciones globales mientras configuramos el timer

  // Configuración del Timer1
  TCCR1A = 0;               // Modo normal
  TCCR1B = 0;               // Limpiamos los registros de control del timer
  TCNT1 = 0;                // Inicializamos el contador a 0
  
  // Calculo del valor para que dispare cada 100 ms (con prescaler de 64)
  // F_CPU = 16 MHz, Tiempo = (prescaler * valor de OCR1A) / F_CPU
  // valor de OCR1A = (Tiempo * F_CPU) / prescaler
  // OCR1A = (0.1s * 16000000) / 64 = 25000
  OCR1A = 2500;            // Establecemos el valor de comparación para 10 ms
  
  // Configuración del temporizador
  TCCR1B |= (1 << WGM12);   // Modo CTC (Clear Timer on Compare Match)
  TCCR1B |= (1 << CS11) | (1 << CS10);  // Prescaler de 64
  
  // Habilitar la interrupción de comparación del Timer1
  TIMSK1 |= (1 << OCIE1A);  // Habilitar la interrupción en comparación con OCR1A

  sei(); // Activa las interrupciones globales
}

// ISR para temporizador
ISR(TIMER1_COMPA_vect) {
  // Ejecuta el cálculo del PID
  calcularPID();

  // Reiniciar el temporizador para evitar desfases
  TCNT1 = 0;
}

// Interrupción para leer el encoder
void encoderISR() {
  // Contar los pulsos del encoder
  encoderPos++;
}
float contarEncoder(){
  unsigned long count = encoderPos; // Guarda el conteo de pulsos
  encoderPos = 0; // Reinicia el contador de pulsos
  //interrupts(); // Reactiva las interrupciones
  
  // Calcula las RPM
  rpm = (count / 494.0) * 6000.0; // Ajusta 494 PPR según tu configuración
  
  // Imprime las RPM en el monitor serial
  
  return rpm;
}

// Función para calcular el PID
void calcularPID() {
  // Actualizar la posición del encoder
  inputf = contarEncoder();

  input = singleEMAFilter.AddValue(inputf); //filtro valor analógico
  
  // Calcular el error
  float error = setpoint - input;

  float deltaTime = sampleTime;

  // Calcular la integral
  integral += error * deltaTime;

  // Calcular la derivada
  float derivative = (input - lastInput) / deltaTime;

  // Calcular la salida PID
  output = Kp * error + Ki * integral + Kd * derivative;
  o1 = output;

  // Limitar la salida para no exceder los valores permitidos
  //if (output > 255) output = 255;
  //else if (output < -255) output = -255;
  if(output < 0)
  {
    output = 0;
  }
  if(output > 255)
  {
    output = 255;
  }

  // Aplicar la salida al motor
  analogWrite(pinMotorP, abs(output));
  digitalWrite(pinMotorN, LOW);

  // Guardar valores para la siguiente iteración
  lastInput = input;
}

void setup() {
  // Configurar pines
  Serial.begin(9600);
  pinMode(pinMotorP, OUTPUT);
  pinMode(pinMotorN, OUTPUT);
  pinMode(pinEncoder, INPUT);

  // Interrupción externa para el encoder
  attachInterrupt(digitalPinToInterrupt(pinEncoder), encoderISR, RISING);

  // Configurar el temporizador
  setupTimer();

}

void loop() {
  leerSetpoint();
  float errord = setpoint-input;
  if (abs(errord) >= 20) {  // Error grande
    Kp = 2;  // Mayor ganancia proporcional
    Ki = 5;  // Menor ganancia integral
    Kd = 0.001; // Pequeño valor derivativo
  } else if ((abs(errord) < 20 && abs(errord) >=10 )) {  // Error medio
    Kp = 1.5;
    Ki = 0.7;
    Kd = 0.0005;
  } else {  // Error pequeño
    Kp = 0.1;
    Ki = 0.9;
    Kd = 0.0;
  }
  
  Serial.print(input);
  Serial.print(" ");
  Serial.println(setpoint);
}
