#include <SingleEMAFilterLib.h>
SingleEMAFilter<float> singleEMAFilter(0.2);

const int encoderPin = 2; // Pin digital conectado al encoder
volatile unsigned long pulseCount = 0; // Contador de pulsos
unsigned long lastTime = 0; // Último tiempo medido
float rpm = 0; // Variable para almacenar las RPM
float rpmn = 0; // Variable para almacenar las RPM

void setup() {
  pinMode(encoderPin, INPUT); // Configura el pin del encoder como entrada
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulses, RISING); // Configura la interrupción
  digitalWrite(5, HIGH);
  digitalWrite(6, LOW);
  Serial.begin(9600); // Inicializa la comunicación serial
}

void loop() {
  unsigned long currentTime = millis(); // Obtiene el tiempo actual
  

  
  if (currentTime - lastTime >= 10) { // Calcula las RPM cada 10 segundo
    //noInterrupts(); // Desactiva las interrupciones para obtener una lectura precisa
    unsigned long count = pulseCount; // Guarda el conteo de pulsos
    pulseCount = 0; // Reinicia el contador de pulsos
    //interrupts(); // Reactiva las interrupciones
    
    // Calcula las RPM
    rpm = (count / 494.0) * 6000.0; // Ajusta 494 PPR según tu configuración
    
    // Imprime las RPM en el monitor serial
    rpmn = singleEMAFilter.AddValue(rpm); //filtro valor analógico
    //Serial.println(rpmn);
    Serial.println(rpmn);
    lastTime = currentTime; // Actualiza el último tiempo medido
  }
}

void countPulses() {
  pulseCount++; // Incrementa el contador de pulsos en cada interrupción
}
