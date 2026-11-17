# Caminhada do exoesqueleto

Códigos de controle de marcha para exoesqueleto com CANable v2 (SLCAN).

## Organização

- **Raiz:** versão NVIDIA Jetson Nano. Veja [LEIA.txt](LEIA.txt).
- **[Windows/](Windows/):** interface gráfica e controles para Windows. Veja [Windows/LEIA.txt](Windows/LEIA.txt).
- **[Raspberry_v4/](Raspberry_v4/):** versão para Raspberry Pi 4, com GPIO e serviço próprios. Veja [Raspberry_v4/LEIA.txt](Raspberry_v4/LEIA.txt).
- **[referencias/](referencias/):** imagem de origem compartilhada para a curva de marcha.

Cada pasta contém o código, as dependências, o CSV da curva e seu JSON complementar. Execute os comandos dentro da pasta correspondente e siga as instruções locais.

## Reproduzir a curva de referência

O arquivo [referencias/figura_ciclo_marcha.png](referencias/figura_ciclo_marcha.png) é a figura usada como origem. Cada CSV tem 101 amostras, espaçadas em 1% do ciclo. O JSON ao lado de cada CSV registra o método de digitalização, a calibração em pixels, a verificação dos sinais dos motores e o hash da imagem. O caminho da imagem no JSON é relativo ao repositório.

A curva é uma digitalização aproximada do traço central da figura, não dados brutos de sensores. As faixas de incerteza do gráfico não foram digitalizadas. A própria figura não identifica o estudo de origem, participante ou velocidade de marcha; portanto esses dados não podem ser reproduzidos a partir deste repositório.

## Instalação e execução

As instruções e dependências de cada plataforma estão nos arquivos `LEIA.txt` e `requirements.txt` correspondentes. `suporte_marcha.py` carrega o CSV e o JSON locais para montar e validar a curva antes de abrir a CAN.

O serviço systemd e o instalador da Raspberry Pi estão incluídos. O instalador prepara dependências, grupos e serviço, mas não cria uma regra `sudoers`; o desligamento por botão requer configuração local aprovada pelo administrador. Os serviços da Jetson dependiam de caminhos particulares da instalação original, então a Jetson é iniciada pelos comandos documentados em `LEIA.txt`.

## Operação

Mantenha o exoesqueleto apoiado, confira os limites e a configuração da plataforma e não abra a mesma CANable simultaneamente em mais de um programa.




