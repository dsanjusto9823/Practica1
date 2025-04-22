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

## 4.Medidor de frecuencia: 
Lo unico que hacemos en este es cambiar elpin de salida por uno que este libre y medir con el osciloscopio la frecuencia maxima tanto de apagado como de encendido. Tomamos estas en estos cuatro casos:
· Con el envio por el puerto série del mensaje i utilizando las funciones de Arduino
· Con el envio por el puerto série y accedirendo directamente a los registros
· Sin el envio por el puerto série del mensaje i utilizando las funciones de Arduino
· Sin el envio por el puerto série y accedirendo directamente a los registros
```
#define LED_PIN 2  // Cambia el pin según disponibilidad
void setup() {
    Serial.begin(115200); // Iniciar puerto serie
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction((gpio_num_t)LED_PIN, GPIO_MODE_OUTPUT);
}
void loop() {
```
### 4.1 Con el envio por puerto série del mensaje y utilizando las funciones del Arduino:
```
 digitalWrite(LED_PIN, HIGH);
 Serial.println("ON");
 digitalWrite(LED_PIN, LOW);
 Serial.println("OFF");
```
Utilizamos de pin de salida el pin 2 y como frecuencia del osciloscopio tenemos 30 kHz.

### 4.2 - Con el envio por puerto série y accediendo directamente a los registros:**
```
  uint32_t *gpio_out = (uint32_t *)GPIO_OUT_REG;
  *gpio_out |= (1 << LED_PIN);
   Serial.println("ON");
   *gpio_out &= ~(1 << LED_PIN);
   Serial.println("OFF");

```
Seguimos usando el pin 2 y la frecuencia es de **30 kHz**.
### 4.3 - Sin el envio por el puerto série del mensaje i utilizando las funciones de Arduino:
```
digitalWrite(LED_PIN, HIGH);
digitalWrite(LED_PIN, LOW);
```
En el tercer caso la frecuencia es de 1.7 MHz.

### 4.4 - Sin el envio por el puerto série y accediendo directamente a los registros:
```
 uint32_t *gpio_out = (uint32_t *)GPIO_OUT_REG;
 *gpio_out |= (1 << LED_PIN);
 *gpio_out &= ~(1 << LED_PIN);

```
Y en este último caso es de **4.7 MHz**



