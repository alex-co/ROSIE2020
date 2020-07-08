/*
 * Sketch: robo_basico.ino
 *
 * Este é um sketch que apresenta as funcionalidades básicas de um robô 
 * controlado por um Arduino que usa dois motores DC. As funcionalidades
 * devem ser modificadas e ampliadas de acordo com o hardware disponível.
 * Este sketch apresenta métodos para:
 * - Acionamento e controle de motores DC usando uma Ponte H dupla;
 * - Uso de encoders de barreira óptica usando as interrupções no Arduino;
 * - Leitura de múltiplas chaves (id do robô) com uso de pull-up interno;
 * - Configuração alternativa do ADC para a leitura da tensão da bateria.
 * Para a demonstracão das funcionalidades, descomente uma ou mais funcões 
 * dentro da funão principal "loop()".
 */
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
/* Bits de controle de direcão para as Pontes H dos motores A e B *** */

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
// para variáveis atualizadas por rotinas de interrupção (ISR).

typedef struct {
    volatile uint32_t tick_cntr = 0;  // Contador de ticks do Motor
    volatile uint32_t tick_last = 0;  // Timestamp do último acionamento
    volatile uint32_t tick_time = 0;  // Tempo entre acionamentos
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

void     isr_encoder_a( void );         // Funcão que atende a interrupcão do Encoder A
void     isr_encoder_b( void );         // Funcão que atende a interrupcão do Encoder B
uint32_t get_motor_status( void );      // Lê configurações das Pontes H
void     set_motor_status( uint32_t );  // Ajusta as Pontes H (direcão e PWM)
uint32_t get_volt_bat( void );          // Lê tensão da bateria
uint8_t  get_robot_id( void );          // Lê o endereço do robô (dipswitchs)

/* ================================================================== */
void setup() {

    Serial.begin(115200);  // Inicializa Serial (para testes)

    // Inicialização dos pinos do dipswitch ( id do robô )
    pinMode(ROBOT_A0, INPUT_PULLUP);    // Bit 0
    pinMode(ROBOT_A1, INPUT_PULLUP);    // Bit 1
//    pinMode(ROBOT_A0, INPUT);    // Bit 0 (sem pull-up)
//    pinMode(ROBOT_A1, INPUT);    // Bit 1 (sem pull-up)

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
    
    set_motor_status( 0x0 );    // Motores parados e freio elétrico

    // Inicialização dos pinos de leitura dos encoders
    pinMode(IRQ_ENC_A, INPUT);  // Pino de interrupção para o Encoder A
    pinMode(IRQ_ENC_B, INPUT);  // Pino de interrupção para o Encoder B

    // Fundo de escala dos ADCs = 1.1V (para a leitura da tensão da bateria)
    // O robô usa um divisor resistivo como sensor da bateria, de forma que
    // o ADC lê Vbat/10 e dessa forma, mesmo com as baterias totalmente
    // carregadas o valor máximo lido é menor que o fundo de escala de 1.1V
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

    // Leitura da tensão da bateria
    mostra_tensao_bateria();

    // Leitura do id do robô
//    mostra_id_robo();


    // Aciona os motores com velocidades progressivas
//    aciona_motores_com_pwm();


     // Mostra dados registrados pelos encoders
//     mostra_dados_encoders();


}
/* ================================================================== */
/* Mostra no serial monitor a tensão lida das baterias do robô, em Volts.
 */
void mostra_tensao_bateria(){

    float volts = get_volt_bat()/1000.0;
    Serial.print(millis());
    Serial.print("   ");
    Serial.print(volts);
    Serial.println(" V");
    delay(1000);
}


/* ------------------------------------------------------------------- */
/* Mostra no serial monitor a leitura das chaves de identificacão do robô
 */
void mostra_id_robo(){

    int id = get_robot_id();
    Serial.print(millis());
    Serial.print("   ID:");
    Serial.println(id);
    delay(1000);  
}

/* ------------------------------------------------------------------- */
/* Incrementa o PWM dos motores de forma progressiva com pausa de
 * um segundo entre incrementos, aguarda 5 segundos, aplica o freio 
 * elétrico nos motores, aguarda mais 2 segundos e reinicia o ciclo
 * de aceleracão e freio.
 */
void aciona_motores_com_pwm(){

    TMotCtrl mtr;
    
    // Os motores são montados em linha e estão posicionados de forma
    // inversa um em relacão ao outro, desta forma, para o robô se mover
    // em uma determinada direcão os motores precisam ter rotacão também em
    // sentidos inversos. Essa inversão pode ser feita na conexão elétrica,
    // na lógica de acionamento ou no comando de acionamento como aqui é feito.
    
    // Direcão do motor A => Sentido Horário
    mtr.config.dir_motor_A = MOTOR_CW;	
    // Direcão do motor A => Sentido Antihorário
    mtr.config.dir_motor_B = MOTOR_CCW;
    
    // Incrementa o PWM dos motores de forma progressiva com pausa de
    // um segundo entre os incrementos.
    for(int i = 1; i < 10; i++){

        mtr.config.pwm_motor_A += 10;	// Altera o PWM para o motor A
        mtr.config.pwm_motor_B += 10;	// Altera o PWM para o motor B
        set_motor_status(mtr.status);	// Aplica alteracão aos motores
        delay(1000);
        
        // Descomente a linha abaixo para mostrar as variáveis que guardam
        // os dados dos encoders para cada passo de incremento do PWMN.
        //mostra_dados_encoders();
    }
    // Pausa de 5 segundos com velocidade máxima, pára os motores, aguarda
    // mais 2 segundos e reinicia o ciclo de aceleracão.
    delay(5000);
    set_motor_status(0);
    delay(2000);
}

/* ------------------------------------------------------------------- */
/* Usa o serial monitor para mostrar os valores das variáveis que
 * guardam as informacões sobre os dois encoders.
 */
void mostra_dados_encoders(){

    uint32_t tempo = millis();
    
    Serial.print(tempo);
    Serial.print("   Pulsos A: ");
    Serial.print(enc_A.tick_cntr);
    Serial.print("   Tempo entre Pulsos: ");
    Serial.print(enc_A.tick_time / 1000.0);
    Serial.println(" ms");
    
    Serial.print(tempo);
    Serial.print("   Pulsos B: ");
    Serial.print(enc_B.tick_cntr);
    Serial.print("   Tempo entre Pulsos: ");
    Serial.print(enc_B.tick_time / 1000.0);
    Serial.println(" ms");

    Serial.println();

    delay(200);
}

/* ================================================================== */
/* *********************************************************************
 * Trata pedido de interrupção gerado pelo sensor óptico A
 */
void isr_encoder_a( void ){

    // Incrementa o contador de pulsos
    enc_A.tick_cntr += 1;

    // Timestamp do tempo atual
    uint32_t agora = micros();
    
    // Calcula tempo entre acionamentos (chamadas desta ISR)
    enc_A.tick_time = agora - enc_A.tick_last;
    
    // Atualiza tempo do último acionamento
    enc_A.tick_last = agora;

    // Troca o estado do LED, fazendo-o acender / apagar a cada
    // chamada de interrupcão do Encoder A.
    //
    // É possível perceber que esse trecho de código continua sendo
    // executado cada vez que o sensor óptico é acionado, mesmo que
    // não haja código em execucão dentro de "loop()", demonstrando
    // que as rorinas que atendem as interrupcões são executadas de
    // forma independente do programa principal.
    status_led = !status_led;
    digitalWrite(LED, status_led);
}

/* *********************************************************************
 * Trata pedido de interrupção gerado pelo sensor óptico B
 */
void isr_encoder_b( void ){

    // Incrementa o contador de pulsos
    enc_B.tick_cntr += 1;

    // Timestamp do tempo atual
    uint32_t agora = micros();
    
    // Calcula tempo entre acionamentos (chamadas desta ISR)
    enc_B.tick_time = agora - enc_B.tick_last;
    
    // Atualiza tempo do último acionamento
    enc_B.tick_last = agora;
}

/* *********************************************************************
 * Leitura da configuração dos motores
 * A variável de 32 bits "status" armazena as seguintes informações:
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
 * Os bits significativos da variavel "data" correspondem a:
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

	// Os bits da variável "motor.status" são mapeadas para as variáveis
	// de "motor.config" (propriedade da estrutura "union") na ordem:
	// motor.config.pwm_motor_B = motor.status (bits 0-7)
	// motor.config.pwm_motor_A = motor.status (bits 8-15)
	// motor.config.dir_motor_B = motor.status (bits 16 e 17)
	// motor.config.dir_motor_A = motor.status (bits 18 e 19)
    motor.status = data;
    
    // Desabilita ponte H
    digitalWrite(HBRID_EN, LOW);

    // Lê bits individuais das variáveis que aguardam as direcões
    // dos motores e escreve os bits de direção nas duas pontes H
    digitalWrite(MTR_AIN1, bitRead(motor.config.dir_motor_A, 0));
    digitalWrite(MTR_AIN2, bitRead(motor.config.dir_motor_A, 1));
    digitalWrite(MTR_BIN1, bitRead(motor.config.dir_motor_B, 0));
    digitalWrite(MTR_BIN2, bitRead(motor.config.dir_motor_B, 1));
    
    // Ajusta valores de PWM para as duas pontes H
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

	// A leitura do ADC corresponde a 1/10 da tensão da bateria,
	// então são somadas 10 leituras consecutivas.
    for (int i = 0; i < n; i++){
        adc += analogRead(VOLT_BAT);
        delayMicroseconds(100);
    }
    // Faz a conversão da faixa de valores de leitura do ADC de 10 bits
    // (0 a 1023) para tensão, considerando que fundo de escala é de 1.1V
    // e multiplica por 1000 para se obter um valor inteiro, em milivolts.
    uint32_t volt = (uint32_t)((adc*1.1/1023.0)*1000.0);
    return volt;
}


/* *********************************************************************
 * Lê identificação do robô configurada via dip switch
 */
uint8_t get_robot_id( ){
	
	// Lê bits das chaves de id do robô. Como as chaves têm lógica invertida,
	// ou seja, LOW = ligado e HIGH = desligado, a variável que armazena o valor
	// final é preenchida por 1's, os valores das chaves são lidos e concatenados
	// no bit menos significativo e após as leituras, todos os bits são invertidos.

    uint8_t robot_id = 0xFF;
    robot_id = robot_id << 1;
    robot_id |= digitalRead(ROBOT_A1);
    robot_id = robot_id << 1;
    robot_id |= digitalRead(ROBOT_A0);
    robot_id = ~robot_id;

    return robot_id;
}
