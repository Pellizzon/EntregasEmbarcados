# Simulado Avaliação prática 1

Esse é um simulado de como será a avaliação prática aplicada na disciplina de Computação Embarcada. 

Sugerimos você fazer como se fosse a avaliação de verdade, assim terá uma ideia de como está indo na disciplina. Essa avaliação cobre os objetivos de aprendizagem da disciplina.

Nessa avaliação iremos trabalhar com o módulo OLED1 (botões/ LEDs e Display), o mesmo já está disponível no repositório da avaliação. 

- Atualize o arquivo `ALUNO.json` com seu nome e e-mail e faça um commit imediatamente.

>  Faça o seu trabalho de maneira ética!

- **A cada 30 minutos você deverá fazer um commit no seu código!**
    - Códigos que não tiverem commit a cada 15min ou que mudarem drasticamente entre os commits serão descartados (conceito I) !!
    - Você deve inserir mensagens condizentes nos commits!
- Duração: 2h

## C

Esse repositório possui o código exemplo do OLED1, você deve editar esse projeto.

### Botões e LEDs

- Configurar os três botões do OLED1 com interrupção e função de callback, sendo:
    - Button0: Subida 
    - Button1: Descida
    - Button2: Subida
    
- Configurar os três LEDs do módulo OLED1 como saída, começando:
    - LED1: Aceso
    - LED2: Apagado
    - LED2: Aceso

### Piscando - TC

Para cada LED do OLED, você deve associar um TimerCounter (TC), que toda vez que acontecer uma interrupção deve fazer com que os LEDs pisquem. Cada um na sua frequência:

- LED1: 5Hz
- LED2: 10Hz
- LED3: 1Hz

> [Veja como tem que ficar](https://www.dropbox.com/s/rmji5q8ylt88qn6/1a.mp4?dl=0)

### Controlando os LEDs - Botões

Toda vez que um botão associado a um LED for apertado você deve fazer com que esse LED pare de piscar, caso o botão seja apertado novamente esse LED deve voltar a piscar:

- SW1 -> LED1
- SW2 -> LED2
- SW3 -> LED3

> [Veja como tem que ficar](https://www.dropbox.com/s/syb9qor473dk7nw/2a.mp4?dl=0)

### Piscando e parando - RTT

Utilize o RTT para fazer com que todos os LEDs pisquem por aprox 5 segundos e depois parem de piscar por mais 5 segundos, e assim por diante.

> [Veja como tem que ficar](https://www.dropbox.com/s/2cv83pnpr23xh2y/3a.mp4?dl=0)

### OLED1

Exiba a frequência na qual cada LED está piscando no OLED1, da seguinte maneira:

> [Veja como tem que ficar](https://www.dropbox.com/s/b0q6usnyt799i00/4a.mp4?dl=0)

### SleepMode

Faça o microcontrolador entrar em sleepmode.
 
## C+

- [ ] Usa flags para comunicar `IRQ` com main
- [ ] Não pisca LED dentro de Handler

## B

- [ ] Mostra barra no OLED indicando o tempo do RTT

## A

- [ ] Usar RTC e exibir o horário no OLED1
