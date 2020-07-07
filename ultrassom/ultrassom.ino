/*
 * Ultrasonic by Erick Simões
 * https://github.com/ErickSimoes/Ultrasonic
 */

// Carrega a biblioteca do sensor HC-SR04
#include <Ultrasonic.h>

#define SSPEED 115200   // Velocidade da interface serial
#define TRIGGER  4      // Pino para o trigger
#define ECHO     5      // Pino para o echo
 
// Inicializa o sensor nos pinos definidos
Ultrasonic ultrasonic(TRIGGER, ECHO);

uint16_t distancia;

/* ********************************************************* */
void setup(){

    Serial.begin(SSPEED);

    // Timeout de 40ms equivale a aprox. 6,8m
    ultrasonic.setTimeout(40000UL);
}

/* ********************************************************* */
void loop(){

    // Le as informacoes do sensor, em cm
    distancia = ultrasonic.read(CM);
    
    // Exibe informacoes no serial monitor
    Serial.print(millis());
    Serial.print(": Dist: ");
    Serial.print(distancia);
    Serial.println("cm");

    delay(1000);
}
/* ********************************************************* */
