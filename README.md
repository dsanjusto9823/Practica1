## Practica1: BLINK 
El objetivo de la practica es el parpadeo parcial del LED usando el microcontrolador ESP32

## 1.Codigo Básico:
```
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

```

## 2.Modificar el programa para que incluya (ON, OFF)
```
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

```

Hemos agregado dos puertos de salida el ON y el OFF, se ha realizado con las funciones que indicadba la practica, el Serial.println. A parte tambien hemos modficado el tiempo de espera del
led de 500 a 1000 milisegundos. 

## 3.Modificar el programa para que actue directamente sobre los registros de los puertos de entrada y salida
```
#include <Arduino.h>
#define LED_PIN 2
#define DELAY 1000
void setup() {
    // Configurar el pin como salida directamente en el hardware
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction((gpio_num_t)LED_PIN, GPIO_MODE_OUTPUT);
    // Inicializar el puerto serie
    Serial.begin(115200);
}
void loop() {
    uint32_t *gpio_out = (uint32_t *)GPIO_OUT_REG; // Puntero al registro GPIO
    *gpio_out |= (1 << LED_PIN);  // Encender LED
    Serial.println("ON");         // Mensaje serie
    delay(DELAY);
    *gpio_out &= ~(1 << LED_PIN); // Apagar LED
    Serial.println("OFF");        // Mensaje serie
    delay(DELAY);
}

```
Se ha modificado para que actue sobre los registros tanto de entrada como de salida, se ha realizado a partir de las funciones que indicaba la practica, el gpio_out. 
