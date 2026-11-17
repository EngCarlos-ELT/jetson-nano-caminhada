# Caminhada do exoesqueleto

Códigos de controle de marcha para exoesqueleto com CANable v2 (SLCAN).

## Organização

- **Raiz:** versão NVIDIA Jetson Nano. Veja [LEIA.txt](LEIA.txt).
- **[Windows/](Windows/):** interface gráfica e controles para Windows. Veja [Windows/LEIA.txt](Windows/LEIA.txt).
- **[Raspberry_v4/](Raspberry_v4/):** versão para Raspberry Pi 4, com GPIO e serviço próprios. Veja [Raspberry_v4/LEIA.txt](Raspberry_v4/LEIA.txt).

Cada pasta mantém os códigos e as dependências da respectiva plataforma. Execute os comandos dentro da pasta correspondente e siga as instruções locais.

## Arquivos locais necessários

Os módulos `suporte_marcha.py`, os utilitários `medir_batentes.py`, as versões `passo_por_botao_coxa_joelho.py` e as curvas CSV acompanham cada plataforma. O arquivo JSON complementar não foi publicado porque contém metadados locais e detalhes de validação física. Como `suporte_marcha.py` carrega esse JSON, os controladores que dependem dele precisam recebê-lo localmente antes da execução.

Os serviços systemd da Jetson, que usam caminhos privados de instalação, não foram publicados. O instalador automático da Raspberry Pi e a regra de desligamento por `sudoers` também não foram publicados. O serviço `Raspberry_v4/exo-master.service` pode ser analisado e instalado manualmente. O desligamento por botão depende de uma configuração local de privilégios revisada pelo administrador.

## Operação

Mantenha o exoesqueleto apoiado, confira limites e configuração da plataforma e não abra a mesma CANable simultaneamente em mais de um programa.




