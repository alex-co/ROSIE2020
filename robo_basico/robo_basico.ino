
/* ****************************************************************** */
/* Estas são as conexões de hardware mapeadas aos pinos do Arduino ** */

#define LED         4      // Led conectado a uma saída digital

#define ROBOT_A0   A4      // Bit 0 do id do robô (LOW = ligado)
#define ROBOT_A1   A5      // Bit 1 do id do robô (LOW = ligado)

#define IRQ_ENC_A   2      // Pino de interrupção do Encoder A
#define IRQ_ENC_B   3      // Pino de interrupção do Encoder B

#define HBRID_EN    6      // Habilita a ponte H (High)
#define MTR_AIN1   A2      // Bit 0 - Controle da ponte H do Motor A
#define MTR_AIN2   A3      // Bit 1 - Controle da ponte H do Motor A
#define MTR_BIN1   A1      // Bit 0 - Controle da ponte H do Motor B
#define MTR_BIN2   A0      // Bit 1 - Controle da ponte H do Motor B
#define MTR_PWMA    9      // Sinal de PWM para controle  do Motor A
#define MTR_PWMB   10      // Sinal de PWM para controle  do Motor B

#define VOLT_BAT   A7      // Tensão da bateria -> Vcc/10


/* ****************************************************************** */
/* Bits de controle das duas Pontes H dos motores A e B ************* */

#define MOTOR_CW     2   // Sentido horário:      10b
#define MOTOR_CCW    1   // Sentido anti-horário: 01b
#define MOTOR_BRK_L  0   // Freio elétrico:       00b
#define MOTOR_BRK_H  3   // Freio elétrico:       11b


/* ****************************************************************** */
/* Estrutura de dados que armazena dos dados dos motores ************ */

typedef union {
    struct {
        uint8_t  pwm_motor_B;        // (bits 0-7)   8 bits: Valor do PWM do Motor B
        uint8_t  pwm_motor_A;        // (bits 8-15)  8 bits: Valor do PWM do Motor A
        uint8_t  dir_motor_B : 2,    // (bits 16-17) 2 bits: BIN1 e BIN2  da ponte H
                 dir_motor_A : 2,    // (bits 18-19) 2 bits: AIN1 e AIN2  da ponte H
                 ign1_4b     : 4;    // (bits 20-23) 4 bits não utilizados (padding)
        uint8_t  ign2_8b;            // (bits 24-31) 8 bits não utilizados (padding)
    } config;
    uint32_t status = 0;             // Leitura/Escrita simuntânea do conjunto de variáveis.
} TMotCtrl;

// status -> | xxxxxxxx | xxxx dir_motor_A dirmotor_B | pwm_motor_A | pwm_motor_b |
//            ----------  ---------------------------- -------------  ------------
//           | byte 3   | byte 2                      | byte 1      | byte 0      |

// "motor" é uma variável do tipo TMotCtrl conforme definido acima
TMotCtrl motor;   

/* ****************************************************************** */
/* Estrutura de dados que armazena dos dados dos encoders *********** */

// Uma variável volatile indica ao compilador que a variável pode ser 
// modificada sem o conhecimento do programa principal. Recomendável
// para variáveis atualizadas por rotinas de interrupção.

typedef struct {
    volatile uint32_t tick_cntr = 0;  // Contador de ticks do Motor
    volatile uint32_t tick_last = 0;  // Timestamp do último tick do Motor
    volatile uint32_t tick_time = 0;  // Tempo entre ticks do Motor
    volatile uint16_t speed     = 0;  // Velocidade instantânea (em mm/s)
    volatile uint8_t  overflow  = 0;  // Núm. de overflows do contador
    volatile uint8_t  new_speed = 0;  // Atualização da vel. instantânea
} TEncCtrl;

// "enc_A" e "enc_B" são variáveis do tipo TEncCtrl conforme definido acima
TEncCtrl  enc_A, enc_B; 

/* ****************************************************************** */
/* Variáveis globais diversas *************************************** */

volatile uint8_t status_led = LOW; // Controla o LED (LOW apagado / HIGH aceso)


/* ******************************************************************* */
/* *** Protótipos das funções **************************************** */
//
// Obs: Este bloco não é necessário para compilação mas é útil como
//      referência durante o processo de desenvolvimento.

void     isr_encoder_a( void );         // ISR do Encoder A
void     isr_encoder_b( void );         // ISR do Encoder B
uint32_t get_motor_status( void );      // Lê configurações das Pontes H
void     set_motor_status( uint32_t );  // Ajusta as Pontes H (dir e PWM)
uint32_t get_volt_bat( void );          // Lê tensão da bateria
uint8_t  get_robot_id( void );          // Lê o endereço do robô (dipswitch)

/* ================================================================== */
void setup() {

    Serial.begin(115200);  // Inicializa Serial (para testes)

    // Inicialização dos pinos do dipswitch ( id do robô )
    pinMode(ROBOT_A0, INPUT_PULLUP);    // Bit 0
    pinMode(ROBOT_A1, INPUT_PULLUP);    // Bit 1

    // Inicialização do pino do LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    // Inicialização dos pinos de controle da Ponte H
    pinMode(HBRID_EN, OUTPUT);     // Habilita ponte H
    digitalWrite(HBRID_EN, HIGH);  // 
    
    pinMode(MTR_AIN1, OUTPUT);  // Bit 0 - Controle da ponte H do Motor A
    pinMode(MTR_AIN2, OUTPUT);  // Bit 1 - Controle da ponte H do Motor A
    pinMode(MTR_BIN1, OUTPUT);  // Bit 0 - Controle da ponte H do Motor B
    pinMode(MTR_BIN2, OUTPUT);  // Bit 1 - Controle da ponte H do Motor B
    pinMode(MTR_PWMA, OUTPUT);  // Sinal de PWM para controle  do Motor A
    pinMode(MTR_PWMB, OUTPUT);  // Sinal de PWM para controle  do Motor B
    
    set_motor_status( 0x0 );            // Motores parados e freio elétrico

    // Inicialização dos pinos de leitura dos encoders
    pinMode(IRQ_ENC_A, INPUT);  // Pino de interrupção para o Encoder A
    pinMode(IRQ_ENC_B, INPUT);  // Pino de interrupção para o Encoder B

    // Fundo de escala dos ADCs = 1.1V (para a leitura da tensão da bateria)
    // Opções: DEFAULT    5V
    //         INTERNAL 1.1V
    //         EXTERNAL Tensão aplicada ao pino AREF (0 a 5V) 
    analogReference(INTERNAL);  // Referência dos ADCs -> 1.1V

    // Habilita interrupção para o Encoder A no pino 2 do Arduino
    pinMode(IRQ_ENC_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IRQ_ENC_A), isr_encoder_a, FALLING);

    // Habilita interrupção para o Encoder B no pino 3 do Arduino
    pinMode(IRQ_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IRQ_ENC_B), isr_encoder_b, FALLING); 
}

/* ================================================================== */
void loop() {

//    teste_1();

}
/* ================================================================== */

void teste_1(){

    TMotCtrl mtr;
    
    mtr.config.dir_motor_A = MOTOR_CW;
    mtr.config.dir_motor_B = MOTOR_CCW;
    
    for(int i = 1; i < 10; i++){

        mtr.config.pwm_motor_A += 10;
        mtr.config.pwm_motor_B += 10;
        set_motor_status(mtr.status);
        delay(1000);
    }
    delay(5000);
    set_motor_status(0);
    delay(2000);
}

/* ================================================================== */
/* *********************************************************************
 * Trata pedido de interrupção gerado pelo sensor óptico A
 */
void isr_encoder_a( void ){

    noInterrupts();

    status_led = !status_led;
    digitalWrite(LED, status_led);
    interrupts();
}

/* *********************************************************************
 * Trata pedido de interrupção gerado pelo sensor óptico B
 */
void isr_encoder_b( void ){

    noInterrupts();

    interrupts();
}

/* *********************************************************************
 * Leitura da configuração dos motores
 * A variável "status" armazena as seguintes informações:
 * PWM  Motor B = 8 bits (bits 0-7)  => 0 a 255  (valores de 0x0 a 0xFF)
 * PWM  Motor A = 8 bits (bits 8-15) => 0 a 255  (valores de 0x0 a 0xFF)
 * Dir. Motor B = 2 bits (bits 16 e 17) => 0 a 3 (valores de 0x0 a 0x3)
 * Dir. Motor A = 2 bits (bits 18 e 19) => 0 a 3 (valores de 0x0 a 0x3)
 * 12 bits (bits 20-31) não utilizados (padding)
 */
uint32_t get_motor_status( ){

    return motor.status;
}

/* *********************************************************************
 * Altera diretamente a configuração de direção e PWM dos motores
 * Atende as mensagens tipo 'M', com dados ocupando 20 bits:
 * PWM  Motor B = 8 bits (bits 0-7)  => 0 a 255  (valores de 0x0 a 0xFF)
 * PWM  Motor A = 8 bits (bits 8-15) => 0 a 255  (valores de 0x0 a 0xFF)
 * Dir. Motor B = 2 bits (bits 16 e 17) => 0 a 3 (valores de 0x0 a 0x3)
 * Dir. Motor A = 2 bits (bits 18 e 19) => 0 a 3 (valores de 0x0 a 0x3)
 * -> Os bits de direção obedecem à seguinte configuração: 
 *    10 -> Motores giram no sentido horário (frente arbitrada) 
 *    01 -> Motores giram no sentido anti-horário
 *    00 -> Condição de freio elétrico (com o GND)
 *    11 -> Condição de freio elétrico (com o Vcc)
 */
void set_motor_status( uint32_t data ) {

    motor.status = data;
    
    // Desabilita ponte H
    digitalWrite(HBRID_EN, LOW);

    // Escreve bits de direção na ponte H
    digitalWrite(MTR_AIN1, bitRead(motor.config.dir_motor_A, 0));
    digitalWrite(MTR_AIN2, bitRead(motor.config.dir_motor_A, 1));
    digitalWrite(MTR_BIN1, bitRead(motor.config.dir_motor_B, 0));
    digitalWrite(MTR_BIN2, bitRead(motor.config.dir_motor_B, 1));
    
    // Escreve bits de PWM na ponte H
    analogWrite(MTR_PWMA, motor.config.pwm_motor_A);
    analogWrite(MTR_PWMB, motor.config.pwm_motor_B);

    // (re)Habilita ponte H
    digitalWrite(HBRID_EN, HIGH);
}

/* *********************************************************************
 * Leitura da tensão da bateria ( Divisor resistivo: Vcc / 10 )
 * Valor de retorno em milivolts
 */
uint32_t get_volt_bat( ) {

    uint8_t  n   = 10;
    uint32_t adc =  0;

    for (int i = 0; i < n; i++){
        adc += analogRead(VOLT_BAT);
        delayMicroseconds(100);
    }
    uint32_t volt = (uint32_t)((adc*1.1/1023.0)*1000.0);
    return volt;
}


/* *********************************************************************
 * Lê identificação do robô configurada via dip switch
 */
uint8_t get_robot_id( ){

    uint8_t robot_id = 0xFF;
    robot_id = robot_id << 1;
    robot_id |= digitalRead(ROBOT_A1);
    robot_id = robot_id << 1;
    robot_id |= digitalRead(ROBOT_A0);
    robot_id = ~robot_id;

    return robot_id;
}
