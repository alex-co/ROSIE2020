/*
 * https://www.arduino.cc/en/Reference/Servo
 */

#include <Servo.h>  // Biblioteca Servo

#define SRVPIN  8

// Inicializa um objeto do tipo Servo
Servo servo_1;
                
uint8_t angulo = 0;    

/* ********************************************************* */
void setup() {

    // Conecta o pino de sinal do servo ao pino do arduino
    servo_1.attach(SRVPIN);

    // Inicia motor na posição zero
    servo_1.write(0); 
}
/* ********************************************************* */
void loop() {

    // Move o servo de 0 grau até 180 graus
    for( angulo = 0; angulo < 180; angulo++ ){                                  

        // Rotaciona o servo em o ângulo específico
        servo_1.write(angulo);
        delay(15);                       
    }
 
    delay(1000);

    // Move o servo de 180 graus até 0 grau
    for( angulo = 180; angulo >= 0; angulo-- ){                                

        // Rotaciona o servo em o ângulo específico
        servo_1.write(angulo);
        delay(15);
    } 

    delay(1000);

}
/* ********************************************************* */
