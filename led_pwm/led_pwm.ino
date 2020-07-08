/*
 * Sketch: led_pwm.ino 
 * 
 * Faz o controle sequencial de três LEDs usando a modulacão por largura
 * de pulso, PWM. Mostra também uma forma alternativa de fazer um atraso
 * em um trecho de código sem o uso da funcão delay().
 */

// ATENCÃO: Muito cuidado ao utilizar os pinos de I/O como
// alimentacão. Cada pino digital pode fornecer até 40mA
// e valores de corrente superiores a este podem danificar
// o microcontrolador.
#define ANODO  8    // Alimentacão dos LEDs 5V

#define LED_R  9    // Catodo do LED vermelho
#define LED_G 10    // Catodo do LED verde
#define LED_B 11    // Catodo do LED azul

int pwm_min = 200;  // Valor mínimo para o PWM
int pwm_max = 250;  // Valor máximo para o PWM

// Passo de incremento/decremento de brilho para os LEDs
int inc_brilho_r = 5;   
int inc_brilho_g = 5;
int inc_brilho_b = 5;

// Guarda valores dos PWMs para os LEDs
int brilho_led_r = pwm_min;
int brilho_led_g = pwm_min;
int brilho_led_b = pwm_min;

// Tempo em ms entre cada passo de incremento ou decremento
int pausa_led = 100;

// Variáveis auxiliares para armazenar valores de tempo
unsigned long pausa_led_r = 0;
unsigned long pausa_led_g = 0;
unsigned long pausa_led_b = 0;

/* ********************************************************* */
void setup() {

    pinMode(LED_R, OUTPUT);		// Inicializa pino conectado ao terminal
    digitalWrite(LED_R, HIGH);	// catodo do LED vermelho (HIGH => desligado)

    pinMode(LED_G, OUTPUT);		// Inicializa pino conectado ao terminal
    digitalWrite(LED_G, HIGH);	// catodo do LED verde (HIGH => desligado)
    
    pinMode(LED_B, OUTPUT);		// Inicializa pino conectado ao terminal
    digitalWrite(LED_B, HIGH);	// catodo do LED azul (HIGH => desligado)
    
    pinMode(ANODO, OUTPUT);		// Inicializa o pino comum dos LEDs (anodo)
    digitalWrite(ANODO, HIGH);	// fornecendo constantemente 5V (nível HIGH).

}

/* ********************************************************* */
void loop() {

	/* Descomente somente uma das funcões abaixo por vez
	 * 
	 * brilho_com_delay() - Usa a funcão delay() ao final do trecho de
	 * código que controla cada LED e fornece uma forma visual de perceber
	 * o efeito do acúmulo de atrasos pelo uso sequencial de delay().
	 * 
	 * brilho_sem_delay() - Mostra uma forma alternativa de fazer um atraso
	 * em um trecho de código sem o uso da funcão delay(), que é bloqueante.
	 * Desta forma todo o trecho fora do bloco de código delimitado pelo "if"
	 * continua executando intermitentemente, sem precisar esperar liberar
	 * o atraso imposto por um suposto delay() dentro deste bloco. 
	 */

    brilho_com_delay();
//    brilho_sem_delay();

}
/* ********************************************************* */

/* As funcões brilho_com_delay() e brilho_com_delay() controlam
 * os LEDs R, G e B individualmente, aumentando e diminuindo o
 * brilho dos LEDs entre os intervalos definidos nas variáveis
 * "pwm_min" e "pwm_max" com passos de PWM definidos nas variáveis
 * "inc_brilho_(r,g,b)"
 */

void brilho_com_delay(){

    // Controla o LED Vermelho
    analogWrite(LED_R, brilho_led_r);
    brilho_led_r = brilho_led_r + inc_brilho_r;
    if(brilho_led_r <= pwm_min || brilho_led_r >= pwm_max)
        inc_brilho_r = -inc_brilho_r;
    delay(pausa_led);

    // Controla o LED Verde
    analogWrite(LED_G, brilho_led_g);
    brilho_led_g = brilho_led_g + inc_brilho_g;
    if(brilho_led_g <= pwm_min || brilho_led_g >= pwm_max)
        inc_brilho_g = -inc_brilho_g;
    delay(pausa_led);

    // Controla o LED Azul
    analogWrite(LED_B, brilho_led_b);
    brilho_led_b = brilho_led_b + inc_brilho_b;
    if(brilho_led_b <= pwm_min || brilho_led_b >= pwm_max)
        inc_brilho_b = -inc_brilho_b;
    delay(pausa_led);

}


void brilho_sem_delay(){

    // Controla o LED Vermelho
    if((millis() - pausa_led_r) > pausa_led){
        pausa_led_r = millis();
        
        analogWrite(LED_R, brilho_led_r);
        brilho_led_r = brilho_led_r + inc_brilho_r;
        if(brilho_led_r <= pwm_min || brilho_led_r >= pwm_max)
            inc_brilho_r = -inc_brilho_r;
    }
    
    // Controla o LED Verde
    if((millis() - pausa_led_g) > pausa_led){
        pausa_led_g = millis();

        analogWrite(LED_G, brilho_led_g);
        brilho_led_g = brilho_led_g + inc_brilho_g;
        if(brilho_led_g <= pwm_min || brilho_led_g >= pwm_max)
            inc_brilho_g = -inc_brilho_g;
    }
    
    // Controla o LED Azul
    if((millis() - pausa_led_b) > pausa_led){
        pausa_led_b = millis();

        analogWrite(LED_B, brilho_led_b);
        brilho_led_b = brilho_led_b + inc_brilho_b;
        if(brilho_led_b <= pwm_min || brilho_led_b >= pwm_max)
            inc_brilho_b = -inc_brilho_b;
    }
}
