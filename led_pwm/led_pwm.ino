
#define ANODO  8

#define LED_R  9
#define LED_G 10
#define LED_B 11

int pwm_min = 200;
int pwm_max = 250;

int inc_brilho_r = 5;
int inc_brilho_g = 5;
int inc_brilho_b = 5;

int brilho_led_r = pwm_min;
int brilho_led_g = pwm_min;
int brilho_led_b = pwm_min;

int pausa_led = 100;

unsigned long pausa_led_r = 0;
unsigned long pausa_led_g = 0;
unsigned long pausa_led_b = 0;

/* ********************************************************* */
void setup() {

    pinMode(LED_R, OUTPUT);
    digitalWrite(LED_R, HIGH);

    pinMode(LED_G, OUTPUT);
    digitalWrite(LED_G, HIGH);
    
    pinMode(LED_B, OUTPUT);
    digitalWrite(LED_B, HIGH);
    
    pinMode(ANODO, OUTPUT);
    digitalWrite(ANODO, HIGH);

}

/* ********************************************************* */
void loop() {

    brilho_com_delay();
//    brilho_sem_delay();
}
/* ********************************************************* */

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

