#include <Wire.h>
#include <Adafruit_MCP4725.h>

// === DAC MCP4725 ===
Adafruit_MCP4725 dac;
const float VREF = 5.0;

// === Pines encoder ===
const int encoderPinA = 19; // INT2
const int encoderPinB = 18; // INT3

// === Pines driver L298N ===
const int enA = 9;
const int in1 = 24;
const int in2 = 22;

// === Entrada analógica de referencia (PWM externo) ===
const int dirInputPin = A14;

// === Variables de velocidad/dirección ===
volatile unsigned long lastTimeA = 0;
volatile float lastSpeed = 0;
volatile int encoderDirection = 1;  // +1 o -1 detectado por el encoder
float pi = 3.14159265;

int inputPWM = 0;
unsigned long startTime = 0;
const unsigned long recordingDuration = 1000000000000000000000; // 10 s CORREGIDO
bool finished = false;

// === Controlador discreto ===
// u(k) = u(k-1) + 1.765*e(k) - 1.127*e(k-1)
volatile float e_k = 0, e_k_1 = 0;
volatile float u_k = 0, u_k_1 = 0;

  int n = 6;
  const float Ts = 0.008f; // 8 ms


  // Dif. atrás + invarianza al impulso (mantengo tus coeficientes)
 //const float b0 = (1.446f + 79.53f * Ts * n);
 //const float b1 = -1.446f;


  // Dif. adelante 
//const float b0 = 1.446f;
//const float b1 = (-1.446f + 79.53f * Ts * n);

//tustin
//const float b0 =(1.446f + 39.765f * Ts * n);
//const float b1 = (-1.446f + 39.765f * Ts * n);

// Invarianza Escalon
const float b0 =(1.446f+(1.446f*79.53f*Ts*n)) ;
const float b1 = -1.446f ;



// Variables auxiliares para referencia
float refSignal = 0;

// === Sincronización DAC con Ts (fuera de ISR) ===
volatile bool dac_ready = false;
volatile uint16_t dac_val_shadow = 0;

// Variables para impresión
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000; // cada 100 ms

void setup() {
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);

  // Configurar interrupciones INT2 e INT3 (flanco descendente)
  EICRA = 0b11110000;
  EIMSK = 0b00001100;

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);


  dac.begin(0x60);

  startTime = millis();

  // === Configurar Timer1 para T = 8 ms ===
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  // Para Ts = 8 ms con prescaler 64:
  // OCR1A = (16,000,000 / 64) * 0.008 - 1 = 1999
 OCR1A = (uint16_t)((16000000.0 / 64.0) * Ts * n) - 1; // 8 ms para prescaler 64 y F_CPU = 16 MHz
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11) | (1 << CS10); // prescaler 64
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

void loop() {
  if (finished) return;

  // Leer voltaje de referencia (no el error)
  float controlVoltage = analogRead(dirInputPin) * (5.0 / 1023.0);

  // Determinar dirección y magnitud según la señal de referencia externa
  int motorDirCmd;
  float magnitude;

  if (controlVoltage >= 2.5) {
    motorDirCmd = -1;
  } else {
    motorDirCmd = 1;
  }

  // Configurar pines de dirección
  if (motorDirCmd > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  // === ACTUALIZACIÓN DEL DAC SINCRONIZADA (mismo timer 8 ms) ===
  if (dac_ready) {
    dac_ready = false;
    dac.setVoltage(dac_val_shadow, false);
  }

  // Finalizar después de 10 s
  if (millis() - startTime >= recordingDuration) {
    noInterrupts();
    analogWrite(enA, 0);
    finished = true;
    dac.setVoltage(0, false);
    interrupts();
  }
}

// === Interrupción canal A (INT2): calcula velocidad y lee canal B para dirección ===
ISR(INT2_vect) {
  unsigned long currentTime = micros();
  if (lastTimeA != 0) {
    lastSpeed = 2 * pi * 1000000.0 / (155.0 * (currentTime - lastTimeA));
  }
  lastTimeA = currentTime;

  // Determinar dirección comparando canal B
  encoderDirection = (digitalRead(encoderPinB) == HIGH) ? 1 : -1;
}

// === Interrupción canal B (INT3): refuerza detección de dirección ===
ISR(INT3_vect) {
  encoderDirection = (digitalRead(encoderPinA) == HIGH) ? -1 : 1;
}

// === Interrupción del Timer1 cada 8 ms - DAC SINCRONIZADO ===
ISR(TIMER1_COMPA_vect) {
  // Leer la referencia (señal de A0)
  refSignal = ((analogRead(dirInputPin) * (5.0 / 1023.0)) - 2.9) * 4.2;

  // Calcular el error entre referencia y velocidad medida
  e_k = refSignal - (lastSpeed / 2.1);  // escalar velocidad a 0–5V aprox.

  // Nueva ecuación en diferencias: u(k) = u(k-1) + 1.765*e(k) - 1.127*e(k-1)
  u_k = u_k_1 + b0 * e_k + b1 * e_k_1;

  // Saturación
  if (u_k > 255) u_k = 255;
  if (u_k < 0)   u_k = 0;
  
  analogWrite(enA, (int)(u_k * 255 / 10));
  
  // === ACTUALIZACIÓN DEL DAC EN EL MISMO TIMER (8 ms) ===
  float maxSpeed = 21.0;
  int dacValue = (int)((lastSpeed * encoderDirection + maxSpeed) / (2 * maxSpeed) * 4095.0);
  dacValue = constrain(dacValue, 0, 4095);
  dac_val_shadow = dacValue;
  dac_ready = true;
  
  // Actualizar variables
  e_k_1 = e_k;
  u_k_1 = u_k;
}
