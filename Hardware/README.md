# Projeto de Aviônica Icaro_II

Última atualização feita por Gustavo Reis (gustavosr8@gmail.com)

## Introdução 

### Motivação

A equipe Antares de espaçomodelismo da Unicamp tem como um de seus projetos desenvolvidos o foguete Anhangá. Trata-se de um foguete de propulsão sólida com apogeu esperado de 1km. Como parte dos requisitos necessários para ter sucesso na missão, o foguete precisa de um sistema de aviônica que será responsável por coletar dados e tomar decisões previamente programadas durante o voo. 

### Objetivo

Sendo assim, o objetivo deste projeto é desenvolver um sistema embarcado para o foguete, capaz de captar dados referentes ao voo, armazenar esses dados de forma segura e tomar decisões baseadas nas leituras feitas.

### Soluções comerciais

Pelo fato de o sistema precisar ser personalizado, se ajustando ao sistema presente no foguete e aos demais requisitos específicos do projeto, não existem alternativas comerciais que cumpram integralmente o objetivo.

## Especificação
    - Microcontrolador: o dispositivo deverá ser capaz de fazer processamentos lógicos, sendo assim necessário um microcontrolador. Por motivos de acessibilidade e familiaridade, será utilizada a plataforma Arduino no MCU ESP-8266 12F.

    - Dados: com o objetivo de receber dados de telemetria do foguete e armazená-los de forma segura, será utilizado um módulo para cartão micro SD e um cartão de 16GB.

    - Interface: como interface, o dispositivo contará com pinos auxiliares para monitoramento do funcionamento da placa e pinos de seleção de função para alimentação e modo de acionamento da ignição, sendo a seleção feita por meio de jumpers.

    - Ignição: deverá suportar dois modos de acionamento do circuito ignitor: por sinal elétrico e por comando remoto. O modo será escolhido via inserção manual de jumpers antes do lançamento.

    - Alimentação: o sistema deverá ter interface de alimentação, podendo operar com a alimentação interna de 3V3 ou alimentação externa de 5V.


## Testes

    A fim de garantir que o sistema seja seguro e que tudo funcione como esperado, é necessário realizar testes controlados de cada parte do sistema, tentando antecipar possíveis falhas que possam ocorrer. 
    Como o sistema de ignição não pode ter acionamento indesejado, devem ocorrer testes em ambientes adversos, como, por exemplo, com componentes desconectados, e o resultado esperado é que o sistema não ative em nenhuma dessas situações.

## Considerações e melhorias futuras

O sistema foi implementado e integrado no foguete Aurora I, para a LASC 2022. A captura de dados se mostrou muito robusta e o sistema de filtragem destes se demonstrou eficiente. A alimentação também teve um bom desempenho (foram utilizadas 2 baterias 18650 para iginição e uma do mesmo tipo para alimentação da placa) e podem até ser estudadas opções de desempenho energético inferior, porém com menor massa. Como pontos negativos para futuras melhorias cabe destacar a pinagem do buzzer, que era em um pino que se ativava sempre que o microcontrolador bootava, tornando o processo de desenvolvimento e debug um tanto desagradável dado o som estridente do dispositivo. O uso de um relay como acionador da ignição também é um ponto negativo do projeto, já que é um dispositivo eletro-mecânico, e, em presença de muito movimento e alta aceleração, como acontece no foguete, pode ocorrer um acionamento indesejado, causando a falha da missão. Uma solução para isso pode ser implementar um acionamento digital via MOSFET. Um último ponto a ser melhorado é a questão da redundância para garantir o sucesso da missão. Esse problema não tem tem uma solução trivial, e deve ser estudado.
