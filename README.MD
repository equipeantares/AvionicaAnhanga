# Base de Acionamento

## Introdução 

### Motivação

A equipe Antares de espaçomodelismo da Unicamp tem como um de seus projetos desenvolvidos o foguete Aurora. Trata-se de um foguete de propulsão sólida com apogeu esperado de 3km. Como parte dos requisitos necessários para ter sucesso na missão, o foguete precisa de um sistema de aviônica que será responsável por coletar dados e tomar decisões previamente programadas durante o voo. Com o objetivo de monitorar o estado do foguete antes, durante e depois do lançamento, é necessário ter um sistema em solo que seja capaz de receber os dados coletados pela aviônica, permitindo que os presentes tenham informações em tempo real. Além disso, esse sistema deve permitir que o acionador do motor seja ligado de forma segura durante o lançamento, não expondo pessoas a riscos.

### Objetivo

Sendo assim, o objetivo deste projeto é desenvolver um sistema de controle de base para o foguete, o qual deve ser capaz de iniciar o lançamento de forma segura e robusta e capaz de receber e apresentar dados da telemetria do voo colhidos pela aviônica.

### Soluções comerciais

Pelo fato de o sistema precisar ser personalizado, se ajustando ao sistema presente no foguete e aos demais requisitos específicos do projeto, não existem alternativas comerciais que cumpram integralmente o objetivo.

## Especificação
    - Microcontrolador: o dispositivo deverá ser capaz de fazer processamentos lógicos, sendo assim necessário um microcontrolador. Por motivos de acessibilidade e familiaridade, será utilizada a plataforma Arduino no MCU ESP-8266 12F.

    - Comunicação e dados: com o objetivo de receber dados de telemetria do foguete, o dispositivo deverá ter um sistema transceptor, que será implementado por um módulo LoRA E32-915T20D.

    - Interface: como interface, o dispositivo contará com pinos auxiliares para monitoramento do funcionamento da placa e pinos de seleção de função para alimentação e modo de acionamento da ignição, sendo a seleção feita por meio de jumpers.

    - Ignição: deverá suportar dois modos de acionamento do circuito ignitor: por sinal elétrico e por comando remoto. O modo será escolhido via inserção manual de jumpers antes do lançamento.

    - Alimentação: o sistema deverá ter interface de alimentação, podendo operar com a alimentação interna de 3V3 ou alimentação externa de 5V.

## Diagrama

![Esquematico](/Assets/Esquematico.png "Esquematico")

![Layout PCB](/Assets/pcb.png "Layout PCB")

## Pinagem

### Pinout conector Screw bateria 18650

Pino | Sinal
-----|------
1  | VBat
2  | GND

### Pinout conector Screw do Acionamento

Pino | Sinal
-----|------
  1  | NiCr_ISO
  2  | GND_ISO

### Pinout conector Screw Alimentação externa

Pino | Sinal
-----|------
  1  | VExt
  2  | GND

### Pinout conector GPIO aux

Pino | Sinal
-----|------
  1  | DIO1
  2  | DIO2
  3  | 3V3
  4  | GND

### Pinout conector LoRA aux

Pino | Sinal
-----|------
  1  | OutputLora
  2  | InputLora
  3  | 3V3
  4  | GND

### Pinout conector I2C aux

Pino | Sinal
-----|------
  1  | SCL_SLAVE
  2  | SDA_SLAVE
  3  | 3V3
  4  | GND

### Pinout conector SPI aux

Pino | Sinal
-----|------
  1  | MOSI
  2  | CS
  3  | MISO
  4  | SCLK
  5  | GND

### Pinout conector de seleção alimentação LoRA

Pino | Sinal
-----|------
  1  | 3V3
  2  | VLora
  3  | VExt

### Pinout conector Transistor PNP acionamento (interno)

Pino | Sinal
-----|------
  1  | Emissor
  2  | Coletor

### Pinout conector Optoacoplador acionamento (externo)

Pino | Sinal
-----|------
  1  | Coletor
  2  | Emissor

## Testes

    A fim de garantir que o sistema seja seguro e que tudo funcione como esperado, é necessário realizar testes controlados de cada parte do sistema, tentando antecipar possíveis falhas que possam ocorrer. 
    Como o sistema de ignição não pode ter acionamento indesejado, devem ocorrer testes em ambientes adversos, como, por exemplo, com componentes desconectados, e o resultado esperado é que o sistema não ative em nenhuma dessas situações.


## Relatório de funcionamento
	Durante o desenvolvimento do projeto foram encotradas algumas falhas que devem ser corrigidas na próxima versão da aviônica. A primeira delas é referente a pinagem do SPI, que estão incorretamente conectadas nos pinos de flash do microcontrolador, de forma que o módulo do cartão SD não funcione como desejado.
	 A segunda falha é o pino do buzzer, que é um pino que é usado para o boot da placa. Isso faz com que o buzzer fique ativado por todo o tempo durante o upload do código, tornando o processo de desenvolvimento do projeto um tanto irritante. Além disso, ocorreram erros por usar esse pino como saída no código, então, para uma próxima atualização do projeto, seria interessante utilizar uma outra GPIO diferente da 9 para operar este dispositivo.
	 Uma outra melhoria que pode ser feita no projeto é repensar o sitema da ignição da recuperação. A utilização de um dispositivo eletromecânico como o relê faz com que possa haver um acionamento indesejado devido à aceleração e movimentação do foguete. Sendo assim, seria mais seguro utilizar um circuito de acionamento digital, utilizando MOSFETs. Além disso, seria interessante implementar um circuito que fosse capaz de identificar a presença ou não do ignitor. Algo assim foi desenvolvido anteriormente pela equipe e o projeto pode ser acessado nas documentações antigas.

Relatório desenvolvido por: Gustavo S. Reis em 01/12/2022
