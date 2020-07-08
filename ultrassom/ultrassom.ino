/* 
 * Sketch: ultrassom.ino
 * 
 * Demonstra o funcionamento de um sensor de proximidade por ultrassom,
 * com o módulo HC-SR04.
 * Utiliza a biblioteca "Ultrasonic" que não é padrão na IDE do Arduino.
 * Para instalar a biblioteca, na IDE do Arduino basta acessar pelo menu:
 * Ferramentas => Gerenciar Bibliotecas...
 * Na tela que irá se abrir, coloque "HC-SR04" no campo de busca
 * Escolha a opcão "Ultrasonic by Erick Simões"e clique em "instalar".
 * Após a conclusão da instalacão, ela já estará disponível para o uso
 * nos sketchs através da IDE do Arduino.
 */

// Referencia o uso da biblioteca do sensor HC-SR04
#include <Ultrasonic.h>

#define SSPEED   115200   // Velocidade da interface serial
#define TRIGGER       4   // Pino trigger do módulo HC-SR04
#define ECHO          5   // Pino echo do módulo HC-SR04
 
// Inicializa o sensor nos pinos de "trigger" e "echo" definidos
Ultrasonic ultrasonic(TRIGGER, ECHO);

uint16_t distancia;   // Armazena valor da distância em cm

/* ********************************************************* */
void setup(){

    Serial.begin(SSPEED);	// Velocidade da interface serial

    // Determina um timeout de 40ms, que equivale a aprox. 6,8m
    ultrasonic.setTimeout(40000UL);
}

/* ********************************************************* */
void loop(){

    // Lê as informacões do sensor, em cm
    distancia = ultrasonic.read(CM);
    
    // Exibe informacões no serial monitor
    Serial.print(millis());
    Serial.print(": Dist: ");
    Serial.print(distancia);
    Serial.println("cm");

    delay(1000);
}
/* ********************************************************* */
