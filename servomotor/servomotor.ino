/* 
 * Sketch: servomotor.ino
 * 
 * Demonstra uma forma simples de controlar um servomotor, fazendo-o mover
 * de 0 a 180 graus e depois de 180 a 0 grau intermitentemente.
 * Utiliza a biblioteca "Servo" que vem por padrão na IDE do Arduino.
 * https://www.arduino.cc/en/Reference/Servo
 */

#include <Servo.h>  // Inclusão da biblioteca "Servo"

#define SSPEED   115200   // Velocidade da interface serial

#define SRVPIN        8   // Comando do servo (laranja)

// Inicializa um objeto "myservo" do tipo "Servo"
Servo myservo;
                
int angulo = 0;   	// Variável (de controle do usuário) que atualiza o ângulo
					// do servo, sendo criada e inicializada com o valor 0.
					// O valor atual do ângulo do servo pode ser também obtido
					// através do método "myservo.read()"

/* ********************************************************* */
void setup() {

    Serial.begin(SSPEED);	// Inicializa a comunicacão serial

    // Conecta o pino de sinal do servo ao pino do Arduino
    myservo.attach(SRVPIN);

    // Inicia motor na posição zero
    myservo.write(angulo); 
}
/* ********************************************************* */
void loop() {

    // Move o servo de 0 grau até 180 graus
    for( angulo = 0; angulo <= 180; angulo++ ){                                  

        // Rotaciona o servo em o ângulo específico
        myservo.write(angulo);  // Comanda o servo para a posicão "angulo"
        delay(15);              // Espera o servo terminar o movimento
    }
    // Mostra o ângulo atual do servo via Serial Monitor
    mostra_angulo();
    delay(1000);

    // Move o servo de 180 graus até 0 grau
    for( angulo = 180; angulo >= 0; angulo-- ){                                

        // Rotaciona o servo em o ângulo específico
        myservo.write(angulo);  // Comanda o servo para a posicão "angulo"
        delay(15);              // Espera o servo terminar o movimento
    } 
    
    // Mostra o ângulo atual do servo via Serial Monitor
    mostra_angulo();
    delay(1000);
}
/* ********************************************************* */

void mostra_angulo(){

    Serial.print(millis());
    Serial.print("  Angulo: ");
    Serial.print(myservo.read());
    Serial.println(" graus");
}
