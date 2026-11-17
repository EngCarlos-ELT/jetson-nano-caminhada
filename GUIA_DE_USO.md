# Guia de uso dos códigos

Este guia descreve a postura de partida, a calibração dos batentes, a instalação por plataforma e a função de cada script. Leia também o LEIA.txt da plataforma antes de ligar os motores.

## Posição inicial obrigatória

Antes de qualquer programa que comande os motores, coloque o exoesqueleto na posição de pé: as duas pernas retas, alinhadas entre si e formando 90° em relação à caixa de suporte da bateria. Mantenha a estrutura apoiada durante os ajustes.

Os controladores leem a posição dos encoders na partida e usam essa pose como referência de “em pé”. Eles não alinham o exoesqueleto automaticamente com a caixa da bateria. Se a posição inicial estiver errada, o programa pode segurar uma pose errada como se fosse a posição de pé.

Confira a posição depois de cada reinicialização, troca de motor ou ajuste da estrutura. Não deixe outro programa conectado à mesma CANable ao iniciar uma rotina.

## Batentes e calibração dos motores

Os batentes mecânicos limitam o curso e ajudam a proteger o operador. A calibração registra os ângulos desses limites para que os controladores mantenham seus comandos dentro da faixa medida. A proteção mecânica e a calibração precisam estar corretas; uma não substitui a outra.

Faça a calibração inicial antes de liberar o equipamento para uso e repita-a após trocar motores, ajustar articulações, alterar os batentes ou notar mudança na leitura dos ângulos. Faça a medição com o exoesqueleto apoiado, sem uma pessoa usando, e sem outro programa na CANable.

1. Primeiro rode a verificação lógica, que não abre a CAN:
   - Jetson ou Raspberry Pi: python3 medir_batentes.py --checar
   - Windows: py -3 medir_batentes.py --checar
2. Rode medir_batentes.py. A porta padrão é /dev/ttyACM0 no Linux e COM6 no Windows. Para outra porta, informe-a no comando.
3. Siga as oito solicitações: dois extremos para cada motor.
   - ID 1: coxa esquerda, perna para a frente e para trás.
   - ID 2: joelho esquerdo, estendido para a frente e flexionado para trás.
   - ID 3: coxa direita, perna para a frente e para trás.
   - ID 4: joelho direito, estendido para a frente e flexionado para trás.
4. O programa pede o estado e deixa o motor sem torque para a medição manual. Mova cada articulação devagar até encostar no batente correspondente; não force o mecanismo além do limite.
5. Confira no resumo cada leitura e a faixa calculada. Responda “s” para atualizar a tabela MEASURED_BATENT_LIMITS_DEG do suporte_marcha.py da mesma pasta somente quando os oito extremos estiverem corretos. “n” conserva a tabela atual. O registro fica em ensaios/batentes_*/medicao.json.
6. Depois, rode ver_angulos_batente.py para conferir a leitura dos quatro motores nos extremos. Esse programa também se conecta à CANable; execute-o sozinho.

Nos comandos abaixo, substitua a porta se necessário:

    Linux:   python3 medir_batentes.py /dev/ttyACM1
    Windows: py -3 medir_batentes.py COM5
    Linux:   python3 ver_angulos_batente.py /dev/ttyACM1
    Windows: py -3 ver_angulos_batente.py COM5

As pastas ensaios/ são locais e ignoradas pelo Git. Copie os registros separadamente se quiser arquivar ou compartilhar as medições.

## NVIDIA Jetson Nano

Instale as dependências seguindo LEIA.txt e coloque o usuário no grupo dialout. Para as rotinas com botões, configure os pinos depois de cada reinicialização da Jetson:

    sudo python3 configurar_gpios_31_33.py

Os pinos 31 e 33 são entradas; o pino 31 comanda a perna esquerda e o 33 a direita. Rode um controlador por vez.

O requirements.txt da Jetson declara pyserial>=3.4; os demais imports usados por esses três programas são módulos locais do repositório ou da biblioteca padrão do Python. A CANable v2 é aberta diretamente em SLCAN, então não inicie slcand ao mesmo tempo. Para os dois controles por botão, use resistores de 1 kΩ entre os pinos 31/33 e GND; aplique 3,3 V para pressionar. O pino 7/GPIO 216 é o LED verde de pronto.

| Arquivo | Função e uso |
|---|---|
| caminhada_torque_pleno.py | Caminhada automática de cinco passos alternados, sem botões. Use depois de conferir a postura inicial e os batentes: python3 caminhada_torque_pleno.py. Aceita a porta como argumento. |
| passo_por_botao.py | Um passo por comando. O botão da perna ativa leva essa perna à frente enquanto a outra acompanha para trás; o passo termina de volta em pé. Soltar antes de 70% pausa o avanço; depois de 70%, o programa conclui o passo. Execute python3 passo_por_botao.py. |
| passo_por_botao_coxa_joelho.py | Variante em que cada botão move a coxa e flexiona o joelho da mesma perna; a outra coxa segura a pose e o outro joelho fica reto. Execute python3 passo_por_botao_coxa_joelho.py. |
| passo_por_botao_so_coxa.py | Move somente as coxas; os joelhos ficam sem torque. Execute python3 passo_por_botao_so_coxa.py. |
| exo_master.py | Abre o modo de passos por botão e controla o encerramento. Segurar os dois botões por 5 segundos solicita desligamento da Jetson. Execute python3 exo_master.py. |
| configurar_gpios_31_33.py | Configura os pinos 31 e 33 como entradas seguras. Exige sudo e a configuração vale até a próxima reinicialização. |
| ler_gpios_31_33.py | Diagnóstico dos botões: imprime os níveis filtrados dos pinos 31 e 33. Use python3 ler_gpios_31_33.py e encerre com Ctrl+C antes de abrir outro controlador. |
| medir_batentes.py | Mede os dois extremos de cada motor, salva um registro local e pode atualizar a tabela de limites do suporte_marcha.py desta pasta. Use a sequência acima. |
| ver_angulos_batente.py | Mostra os ângulos absolutos dos quatro motores com o torque desligado. Use para conferir os batentes, sem outro programa na CANable. |
| suporte_marcha.py | Biblioteca compartilhada pelos controladores: comunicação SLCAN, leitura da curva CSV/JSON e dados dos limites medidos. Não execute diretamente. |
| limites_operacionais_exo.py | Funções auxiliares de faixa operacional e limite de esforço. Importado pelos controladores; não execute diretamente. |

### passo_por_botao_coxa_joelho.py

**O que faz.** É o modo interativo que comanda coxa e joelho da perna cujo botão foi pressionado. Cada perna age independentemente: a coxa avança e o joelho correspondente flexiona; a outra coxa mantém a posição de pé e o outro joelho fica reto. Os valores iniciais do código são 40° para a coxa e 45° para a flexão do joelho. Ao fim do passo, a perna volta à referência de pé.

Segurar o botão faz o movimento avançar. Soltar antes de 70% pausa a trajetória e mantém o ângulo alcançado; depois de 70%, o programa termina o passo e retorna à pose inicial. Um novo passo no mesmo lado só é aceito depois de soltar o botão. Se o joelho tiver falha, exceder o limite de esforço ou ficar sem progresso, esse joelho é liberado; as coxas continuam. Ctrl+C encerra o programa e envia os comandos para soltar o torque.

**Dependências.** Python 3, pyserial (requirements.txt), CANable v2 em SLCAN e os arquivos locais caminhada_torque_pleno.py, suporte_marcha.py, limites_operacionais_exo.py e ler_gpios_31_33.py, além do CSV/JSON da curva. Os botões usam GPIOs da Jetson, sem pacote Python adicional. Configure o pinmux como descrito acima.

**Como usar.** Com o exo apoiado na posição de pé a 90° especificada neste guia, a CANable livre e os botões conectados aos pinos 31 e 33:

    python3 passo_por_botao_coxa_joelho.py --checar
    python3 passo_por_botao_coxa_joelho.py

O primeiro comando verifica a lógica sem abrir CAN nem GPIO. O segundo inicia o controlador; pino 31/D6 avança a perna esquerda e pino 33/D13 a direita. Um nível de 3,3 V precisa permanecer estável por 100 ms para contar como pressionado; a soltura é confirmada em 30 ms. A porta padrão é /dev/ttyACM0; passe outra como argumento, por exemplo /dev/ttyACM1.

### passo_por_botao_so_coxa.py

**O que faz.** É uma variante de teste que movimenta somente os motores das coxas. Cada botão avança a coxa do seu lado até 40° e a traz de volta à referência de pé; a outra coxa mantém a pose. Os dois joelhos recebem o comando para ficarem sem torque durante toda a execução. Use essa variante somente com a estrutura apoiada, pois os joelhos não sustentam a pose.

O acionamento e a pausa seguem a mesma lógica de confirmação dos botões: 3,3 V estável por 100 ms ativa; soltar antes de 70% pausa no ângulo atual e soltar depois de 70% conclui o passo. Ctrl+C encerra e solta o torque das coxas.

**Dependências.** Python 3, pyserial, CANable v2 em SLCAN, GPIOs 31/33 configurados na Jetson e os módulos locais caminhada_torque_pleno.py, ler_gpios_31_33.py, suporte_marcha.py e limites_operacionais_exo.py, com os arquivos CSV/JSON da curva.

**Como usar.** Depois de conferir postura, batentes e pinos, rode:

    python3 passo_por_botao_so_coxa.py --checar
    python3 passo_por_botao_so_coxa.py

A opção --checar verifica a lógica sem abrir CAN ou GPIO. O comando normal usa /dev/ttyACM0; informe outra porta como argumento, se necessário. Pino 31/D6 comanda a coxa esquerda e pino 33/D13 a direita.

### medir_batentes.py

**O que faz.** Este é o medidor assistido pelo operador. Ele não movimenta os motores até os limites: pede o estado de um motor, envia o quadro que o deixa sem torque e solicita que o operador gire a articulação à mão até o batente indicado. Lê e registra o ângulo daquele extremo. Repete para os dois extremos de cada motor: ID 1 coxa esquerda, ID 2 joelho esquerdo, ID 3 coxa direita e ID 4 joelho direito.

**Dependências.** Python 3, pyserial, CANable v2 em SLCAN e o suporte_marcha.py que fica na mesma pasta. Não usa GPIO nem os controladores de passo. Não rode outro programa ao mesmo tempo na CANable.

**Como usar.** Deixe o exo apoiado, sem uma pessoa usando, calibre um motor por vez e siga as oito solicitações. Mova a junta lentamente até encostar no batente físico; não force além dele. Primeiro valide a lógica, sem abrir a CAN; depois inicie a medição:

    Jetson:     python3 medir_batentes.py --checar
    Jetson:     python3 medir_batentes.py
    Windows:    py -3 medir_batentes.py --checar
    Windows:    py -3 medir_batentes.py
    Raspberry:  python3 medir_batentes.py --checar
    Raspberry:  python3 medir_batentes.py

Para uma porta diferente da padrão, informe-a depois do nome do script: /dev/ttyACM1 no Linux ou COM5 no Windows. No fim, confira os ângulos e a faixa calculada. Responda “s” somente se os oito extremos estiverem corretos; isso atualiza MEASURED_BATENT_LIMITS_DEG no suporte_marcha.py da mesma plataforma. “n” mantém os limites atuais. Em ambos os casos, a medição é salva localmente em ensaios/batentes_*/medicao.json; essa pasta é ignorada pelo Git.

### caminhada_torque_pleno.py

**O que faz.** Executa automaticamente cinco passos alternados seguindo a curva de marcha do CSV, sem depender dos botões GPIO. Usa os quatro motores, com os valores padrão de 18° para o quadril e 45° para a flexão do joelho. A origem de cada junta é a leitura do encoder feita no início; por isso o exo precisa estar fisicamente alinhado na pose de pé antes de iniciar. No fim ou numa parada, o programa envia o comando para sair do modo de posição e espera a resposta antes de fechar a CAN.

O controle usa torque nominal continuamente. Só recorre ao pico por até 0,25 s quando o nominal não faz o movimento progredir; se ainda não houver progresso, reduz a amplitude. Temperatura, velocidade, batentes, falhas ou perda de feedback também podem interromper o movimento. Os valores de torque ficam definidos no arquivo; não os aumente para compensar uma montagem desalinhada.

**Dependências.** Python 3, pyserial (requirements.txt), CANable v2 em SLCAN, caminhada_torque_pleno.py junto de suporte_marcha.py e limites_operacionais_exo.py, mais o CSV/JSON da curva na mesma pasta. Não usa GPIO.

**Como usar.** Com os batentes calibrados, a postura inicial conferida e nenhum outro programa usando a CANable:

    python3 caminhada_torque_pleno.py

A porta padrão é /dev/ttyACM0. Outra porta pode ser informada como argumento, por exemplo python3 caminhada_torque_pleno.py /dev/ttyACM1, ou pela variável CANABLE_PORT. Essa rotina não tem o modo --checar das duas variantes por botão; ao executá-la, ela abre a CANable e pode comandar os motores.

O arquivo exo-desliga.sudoers contém a regra de desligamento do usuário nvidia. Os serviços automáticos da Jetson não estão nesta cópia pública; execute os programas pelos comandos acima.

## Windows

Instale o Python e as dependências com:

    py -3 -m pip install -r requirements.txt

| Arquivo | Função e uso |
|---|---|
| interface_exo.py | Janela principal para ligar a CANable, segurar Esquerda/Direita e retornar à pose Em pé. As setas e A/D comandam as pernas; espaço mantém a pose em pé. Execute py -3 interface_exo.py. Para outra porta, informe-a no comando, por exemplo py -3 interface_exo.py COM5. |
| iniciar.bat | Atalho para abrir a interface principal usando COM6. Dê dois cliques no arquivo. |
| caminhada_torque_pleno.py | Caminhada automática de cinco passos alternados, sem comandos de botão. Execute py -3 caminhada_torque_pleno.py. |
| passo_por_botao.py | Implementação do passo por botão usada pela interface principal. Executada normalmente, abre a interface. Use interface_exo.py para escolher uma porta diferente de COM6. |
| passo_por_botao_coxa_joelho.py | Janela alternativa que comanda coxa e joelho da perna selecionada. Execute py -3 passo_por_botao_coxa_joelho.py. |
| medir_batentes.py | Calibra os dois extremos dos quatro motores e pode atualizar suporte_marcha.py desta pasta. Use a sequência de calibração acima. |
| ver_angulos_batente.py | Lê os ângulos com o torque desligado; use para conferir os limites medidos. |
| gpio_windows.py | Adaptador de botões virtuais da interface. Não abre GPIO físico e não é usado sozinho. |
| suporte_marcha.py | Biblioteca de comunicação CAN, curva e dados dos motores. Não execute diretamente. |
| limites_operacionais_exo.py | Auxilia os controladores a aplicar as faixas medidas. Não execute diretamente. |

A versão Windows não usa os pinos físicos de GPIO. As entradas vêm da janela. Rode apenas um programa por vez com a CANable.

## Raspberry Pi 4

A instalação automatizada prepara pacotes, grupos, a regra sudoers e o serviço:

    sudo bash instalar.sh

Depois de adicionar o usuário aos grupos, saia e entre novamente. O serviço exo-master.service inicia o modo de botões. Para uso manual, pare primeiro o serviço com sudo systemctl stop exo-master.service. A porta padrão é /dev/ttyACM0.

| Arquivo | Função e uso |
|---|---|
| caminhada_torque_pleno.py | Caminhada automática de cinco passos alternados, sem botões. Execute python3 caminhada_torque_pleno.py. |
| exo_master.py | Mestre do modo por botões e do LED de pronto. Segurar os dois botões por 5 segundos solicita o desligamento da Pi. Normalmente é iniciado pelo serviço; para iniciar manualmente, use python3 exo_master.py. |
| passo_por_botao.py | Controlador de passos por botões. Pino físico 31/BCM 6 comanda a perna esquerda; pino 33/BCM 13 comanda a direita. O passo começa e termina em pé. Pare o serviço antes e execute python3 passo_por_botao.py. |
| gpio_raspberry.py | Módulo de leitura e filtragem dos botões e acesso ao LED no pino físico 7/BCM 4. Execute python3 gpio_raspberry.py para ver os níveis das entradas; encerre com Ctrl+C. |
| medir_batentes.py | Mede e grava os limites dos quatro motores na tabela suporte_marcha.py desta pasta. Use a sequência de calibração acima. |
| ver_angulos_batente.py | Mostra os ângulos absolutos dos motores com torque desligado. Use para conferir os batentes. |
| suporte_marcha.py | Biblioteca SLCAN, curva CSV/JSON e calibração usada pelos controladores. Não execute diretamente. |
| limites_operacionais_exo.py | Auxiliares de faixa operacional e controle de esforço. Não execute diretamente. |
| instalar.sh | Instala dependências, grupos, regra sudoers limitada aos comandos de desligamento e serviço. Revise o script antes de executar. |
| exo-master.service | Modelo do serviço systemd; instalar.sh gera uma unidade com o caminho e usuário reais da Pi. |

O pino 7 é apenas o LED. Os pinos 31 e 33 são entradas com pull-down. 0 significa botão solto; 3,3 V firme por 100 ms significa pressionado. Segurar os dois botões juntos impede a caminhada e, no modo exo_master.py, por 5 segundos solicita desligamento da Pi.

## Arquivos da curva e verificações

- referencia_figura_ciclo_marcha.csv: 101 pontos da curva, um por cento do ciclo.
- referencia_figura_ciclo_marcha.json: método, escalas de digitalização e referência à imagem.
- referencias/figura_ciclo_marcha.png: figura de origem compartilhada; os JSON das subpastas apontam para ela com caminho relativo ../referencias/figura_ciclo_marcha.png.
- requirements.txt: dependências Python da plataforma correspondente.

Os programas com a opção --checar verificam lógica sem abrir a CAN e sem comandar motores. Exemplos:

    python3 passo_por_botao.py --checar
    python3 medir_batentes.py --checar
    py -3 interface_exo.py --checar

Essa opção não substitui a conferência dos batentes nem o teste de comunicação e postura antes do uso físico.

