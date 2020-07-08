/*
 * Sketch: ldr.ino
 * 
 * Faz a leitura de um LDR conectado como divisor resistivo com um
 * resistor de 10K. Também calcula uma autoescala de luminosidade,
 * variando de 0% a 100% com base nos valores máximos e mínimos de
 * luminosidade nos quais o LDR é submetido.
 */
 
#define SSPEED 115200   // Velocidade da interface serial

// ATENCÃO: Muito cuidado ao utilizar os pinos de I/O como
// alimentacão. Cada pino digital pode fornecer até 40mA
// e valores de corrente superiores a este podem danificar
// o microcontrolador.
#define VCC    A0   // 5V via pino de I/O (LDR)
#define GND    A2   // 0V via pino de I/O (Resistor)

#define LDR    A1   // Leitura do divisor resistivo (LDR + resistor)

#define LED    13   // Led embarcado na placa

uint16_t valor_adc, valor_min, valor_max;
uint8_t  lum_perc;

/* ********************************************************* */
void setup() {

    // Inicializa interface serrial
    Serial.begin(SSPEED);

    // Inicializa pinos de I/0
    pinMode(VCC, OUTPUT);
    digitalWrite(VCC, HIGH); // 5 Volts
    
    pinMode(GND, OUTPUT);
    digitalWrite(GND, LOW);  // 0 Volts

    pinMode(LDR, INPUT);
    pinMode(LED, OUTPUT);

    // Valor inicial para as variáveis
    valor_min = 1023;   // Inicializa com o máximo valor do ADC
    valor_max = 0;      // Inicializa com o mínimo valor do ADC
    lum_perc  = 0;      // Para a conversão da luminosidade em %
}
/* ********************************************************* */
void loop() {

    // Lê valor do ADC para o LDR
    valor_adc = analogRead(LDR);

    // Atualiza valor mínimo, se necessário
    if( valor_adc < valor_min )
        valor_min = valor_adc;

    // Atualiza valor máximo, se necessário
    if( valor_adc > valor_max )
        valor_max = valor_adc;

    // Transforma o valor atual do ADC para uma escala de 0 a 100
    // tendo como referência os valores mínimos e máximos medidos
    // que são constantemente atualizados de acordo com as condicões
    // de luminosidade que o LDR é submetido.
    lum_perc = map(valor_adc, valor_min, valor_max, 0, 100);

    // Aciona o Led embarcado na placa se a
    // luminosidade for menor do que 50%
    digitalWrite(LED, (lum_perc < 50 ? HIGH : LOW));
    
    // Envia dados para a porta serial
    Serial.print(millis());
    Serial.print(" : LDR ");
    Serial.print(valor_adc);
    Serial.print(" : MIN ");
    Serial.print(valor_min);
    Serial.print(" : MAX ");
    Serial.print(valor_max);
    Serial.print(" : Lum ");
    Serial.print(lum_perc);
    Serial.print("% ");
    Serial.println();

    delay(1000);
}
/* ********************************************************* */
