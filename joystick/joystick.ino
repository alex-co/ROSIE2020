/*
 * Sketch: joystick.ino
 * 
 * Faz a leitura de um joystick, composto por dois potenciômetros
 * (para as leituras dos eixos x e y) e um botão sem resistor de pull-up. 
 */


#define SSPEED 115200   // Velocidade da interface serial

#define TEMPO  250      // Tempo de delay em milisegundos

#define EIXO_X   A0     // Potenciômetro do Eixo X
#define EIXO_Y   A1     // Potenciômetro do Eixo Y
#define BOTAO    A2     // Pino conectado ao botão


uint32_t var_tempo;		// Variável de 32 bits e sem sinal, 
						// auxiliar para contagem de tempo

uint16_t valor_X, valor_Y;      // Guarda valores dos ADCs de 10 bits
uint8_t  valor_B, pwm_A, pwm_B; // Guarda estado do botão e valores de PWM



/* ********************************************************* */
void setup() {

    Serial.begin(SSPEED);			// Inicializa a comunicacão serial

    pinMode(EIXO_X, INPUT);			// Potenciômetro do Eixo X 
    pinMode(EIXO_Y, INPUT);			// Potenciômetro do Eixo Y
    
    pinMode(BOTAO, INPUT_PULLUP);	// Ativa pull-up interno para o pino do botão
    
    var_tempo = 0;	// Inicializa variável auxiliar para contagem de tempo

}
/* ********************************************************* */
void loop() {

    // Efeito de pausa de "TEMPO" milisegundos sem usar delay() 
    if( (millis() - var_tempo) > TEMPO ){
        var_tempo = millis();

        valor_X = analogRead(EIXO_X);	// Lê valor analógico do Eixo X (0 a 5V)
        valor_Y = analogRead(EIXO_Y);	// Lê valor analógico do Eixo Y (0 a 5V)
        valor_B = digitalRead(BOTAO);	// Lê valor digital do botão (0 ou 5V)

        // Exemplo de conversão da faixa de 0-1023 para 0-255
        // Divide por 4 (10 => 8 bits com shift de 2 bits para a direita)
        pwm_A = valor_X >> 2;
        pwm_B = valor_Y >> 2;

        // Inverte valores para aumentar o PWM quando o joystick é
        // movido para a frente (eixo X) e para a direita (eixo Y).
        pwm_A = ~pwm_A;		// Inverte todos os bits das variáveis
        pwm_B = ~pwm_B;		// pwm_A e pwm_B
        
        imprime_serial();
        
    }
  

}
/* ********************************************************* */

void imprime_serial(){

    Serial.print(millis());
    Serial.print("   VALOR X: ");
    Serial.print(valor_X);
    Serial.print(" PWM A: ");
    Serial.print(pwm_A);
    Serial.print("   VALOR Y: ");
    Serial.print(valor_Y);
    Serial.print(" PWM B: ");
    Serial.print(pwm_B);
    Serial.print("   BTN: ");
    Serial.println(valor_B == LOW ? "ON" : "OFF");
}
