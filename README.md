## Practica1: BLINK 
El objetivo de la practica es el parpadeo parcial del LED usando el microcontrolador ESP32

## 1.Codigo Básico:

#include <Arduino.h>
#define LED_BUILTIN 2
#define DELAY 500
void setup() {
pinMode(LED_BUILTIN, OUTPUT);
}
void loop() {
digitalWrite(LED_BUILTIN, HIGH);
delay(DELAY);
digitalWrite(LED_BUILTIN, LOW);
delay(DELAY);
}

## 2.Modificar el programa para que incluya (ON, OFF)

#include <Arduino.h>
#define LED_BUILTIN 2 // Pin del LED
#define DELAY 1000      // Tiempo de espera en milisegundos
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);  // Configurar LED como salida
    Serial.begin(115200);          // Inicializar comunicación serie
}
void loop() {
    digitalWrite(LED_BUILTIN, HIGH);  // Encender LED
    Serial.println("ON");             // Enviar mensaje por serie
    delay(DELAY);                    
    digitalWrite(LED_BUILTIN, LOW);   // Apagar LED
    Serial.println("OFF");            // Enviar mensaje por serie
    delay(DELAY);                    
}

Hemos agregado dos puertos de salida el ON y el OFF, se ha realizado con las funciones que indicadba la practica, el Serial.println. A parte tambien hemos modficado el tiempo de espera del
led de 500 a 1000 milisegundos. 


