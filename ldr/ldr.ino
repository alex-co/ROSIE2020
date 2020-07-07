
#define SSPEED 115200   // Velocidade da interface serial

#define VCC    A0   // 5V via pino de I/O
#define GND    A2   // 0V via pino de I/O
#define LDR    A1   // Leitura do divisor resistivo
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
    valor_min = 1023;
    valor_max = 0;
    lum_perc  = 0;
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
