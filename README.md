## ROSIE2020
Material apresentado no **Minicurso Programação Arduino Aplicada à Robótica Móvel**

ROSIE 2020

### Conteúdo das pastas:

* **Apresentacao** - Slides da apresentação.

* **joystick** - Sketch: *joystick.ino* - Faz a leitura de um joystick, composto por dois potenciômetros (para as leituras dos eixos x e y) e um botão sem resistor de pull-up.

* **ldr** - Sketch: *ldr.ino* - Faz a leitura de um LDR conectado como divisor resistivo com um resistor de 10K. Também calcula uma autoescala de luminosidade,
 variando de 0% a 100% com base nos valores máximos e mínimos de luminosidade nos quais o LDR é submetido. 

* **led_pwm** - Sketch: *led_pwm.ino* - Faz o controle sequencial de três LEDs usando a modulacão por largura de pulso, PWM. Mostra também uma forma alternativa de fazer um atraso em um trecho de código sem o uso da funcão delay().

* **servomotor** - Sketch: *servomotor.ino* - Demonstra uma forma simples de controlar um servomotor, fazendo-o mover de 0 a 180 graus e depois de 180 a 0 grau intermitentemente. Utiliza a biblioteca "Servo" que vem por padrão na IDE do Arduino.

* **ultrassom** - Sketch: *ultrassom.ino* - Demonstra o funcionamento de um sensor de proximidade por ultrassom,com o módulo HC-SR04. Utiliza a biblioteca "Ultrasonic" que não é padrão na IDE do Arduino.

* **robo_baisco** - Sketch: *robo_basico.ino* - Este sketch apresenta as funcionalidades básicas de um robô controlado por um Arduino que usa dois motores DC. 
As funcionalidades devem ser modificadas e ampliadas de acordo com o hardware disponível.
 
   - Acionamento e controle de motores DC usando uma Ponte H dupla;
   - Uso de encoders de barreira óptica usando as interrupções no Arduino;
   - Leitura de múltiplas chaves (id do robô) com uso de pull-up interno;
   - Configuração alternativa do ADC para a leitura da tensão da bateria.
