
#define SSPEED 115200   // Velocidade da interface serial
#define TEMPO  250      // milisegundos

#define EIXO_X   A0
#define EIXO_Y   A1
#define BOTAO    A2


uint32_t var_tempo;

uint16_t valor_X, valor_Y;
uint8_t  valor_B, pwm_A, pwm_B;

void imprime_serial(uint16_t,uint16_t,uint8_t);

/* ********************************************************* */
void setup() {

    Serial.begin(SSPEED);

    pinMode(EIXO_X, INPUT);
    pinMode(EIXO_Y, INPUT);
    pinMode(BOTAO,  INPUT);
    
    var_tempo = 0;

}
/* ********************************************************* */
void loop() {

    if( (millis() - var_tempo) > TEMPO ){
        var_tempo = millis();

        valor_X = analogRead(EIXO_X);
        valor_Y = analogRead(EIXO_Y);
        valor_B = digitalRead(BOTAO);

        // Divide por 4 (10 => 8 bits)
        pwm_A = valor_X >> 2;
        pwm_B = valor_Y >> 2;

        // Inverte valores
        // Aumenta em X para frente e em Y para a direita 
        pwm_A = ~pwm_A;
        pwm_B = ~pwm_B;
        
        imprime_serial(valor_X,valor_Y,valor_B, pwm_A, pwm_B);
    }
  

}
/* ********************************************************* */

void imprime_serial(uint16_t vx, uint16_t vy, uint8_t vb, uint8_t pa, uint8_t pb){

    Serial.print(millis());
    Serial.print(" X: ");
    Serial.print(vx);
    Serial.print(" A: ");
    Serial.print(pa);
    Serial.print(" Y: ");
    Serial.print(vy);
    Serial.print(" B: ");
    Serial.print(pb);
    Serial.print(" BTN: ");
    Serial.println( vb == LOW ? "ON" : "OFF");
}

