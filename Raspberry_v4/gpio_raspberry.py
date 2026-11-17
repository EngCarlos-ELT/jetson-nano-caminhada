#!/usr/bin/env python3
"""GPIO da Raspberry Pi 4, nos mesmos furos do header de 40 pinos.

Pino fisico 31 = BCM 6, botao da perna esquerda.
Pino fisico 33 = BCM 13, botao da perna direita.
Pino fisico 7 = BCM 4, LED verde. Nunca vira entrada.
O chip e o gpiochip0. A Pi 5 usa outro chip e nao entra nesta versao.

Cada botao usa o pull-down interno do BCM. O resistor de 1 kΩ para o GND
do pino 6 pode continuar em paralelo. 0 e repouso. 1 e 3,3 V firme.
O programa nao inverte a leitura. Um pico so vira 1 depois de 100 ms.
A volta para 0 confirma em 30 ms.
"""

import os
import sys
import time


AMOSTRA_S = 0.01
CONFIRMA_UM_S = 0.10
CONFIRMA_ZERO_S = 0.03
PERIODO_TELA_S = 0.2
CHIP_PI4 = 0
BCM_ESQUERDA = 6
BCM_DIREITA = 13
BCM_LED = 4
PINOS = (
    ("pino31_esquerda", BCM_ESQUERDA),
    ("pino33_direita", BCM_DIREITA),
)
_chip = None
_lgpio = None


def exigir(condicao, mensagem):
    if not condicao:
        raise RuntimeError(mensagem)


def modelo_da_placa():
    try:
        with open("/proc/device-tree/model", "rb") as arquivo:
            texto = arquivo.read().decode("ascii", "replace")
    except (IOError, OSError):
        return ""
    return texto.replace("\x00", "").strip()


def pinmux_deste_boot():
    """Na Pi 4 o gpiochip0 substitui o pinmux da Jetson."""
    return os.path.exists("/dev/gpiochip%d" % CHIP_PI4)


class Filtro(object):
    """Comeca em 0. So sobe depois de CONFIRMA_UM_S. Desce mais rapido."""

    def __init__(self):
        self.estavel = 0
        self.bruto = 0
        self.desde = None

    def atualizar(self, bruto, agora):
        if self.desde is None or bruto != self.bruto:
            self.bruto = bruto
            self.desde = agora
        duracao = agora - self.desde
        if bruto == 1 and self.estavel == 0 and duracao >= CONFIRMA_UM_S:
            self.estavel = 1
        elif bruto == 0 and self.estavel == 1 and duracao >= CONFIRMA_ZERO_S:
            self.estavel = 0
        return self.estavel


def _biblioteca():
    global _lgpio
    if _lgpio is not None:
        return _lgpio
    try:
        import lgpio
    except ImportError:
        raise OSError(
            "Falta o lgpio. Na Raspberry Pi OS: sudo apt install python3-lgpio")
    _lgpio = lgpio
    return lgpio


def _abrir():
    global _chip
    if _chip is not None:
        return _chip
    if not pinmux_deste_boot():
        raise OSError("Nao achei /dev/gpiochip%d. Esta versao e da Raspberry Pi 4."
                      % CHIP_PI4)
    placa = modelo_da_placa()
    if placa and not placa.startswith("Raspberry Pi 4 "):
        raise OSError(
            "Esta pasta e so da Raspberry Pi 4. A placa lida foi: %s" % placa)
    lgpio = _biblioteca()
    try:
        _chip = lgpio.gpiochip_open(CHIP_PI4)
    except Exception as exc:
        raise OSError(
            "Nao abri /dev/gpiochip%d (%s). O usuario precisa do grupo gpio."
            % (CHIP_PI4, exc))
    return _chip


def exportar_entrada(numero):
    if numero == BCM_LED or numero not in (BCM_ESQUERDA, BCM_DIREITA):
        raise OSError("BCM %d nao e botao desta versao" % numero)
    lgpio = _biblioteca()
    chip = _abrir()
    try:
        lgpio.gpio_free(chip, numero)
    except Exception:
        pass
    try:
        lgpio.gpio_claim_input(chip, numero, lgpio.SET_PULL_DOWN)
    except lgpio.error as exc:
        raise OSError("Nao reivindiquei o BCM %d como entrada: %s" % (numero, exc))


def ler_bruto(numero):
    if numero not in (BCM_ESQUERDA, BCM_DIREITA):
        raise OSError("BCM %d nao e botao desta versao" % numero)
    lgpio = _biblioteca()
    chip = _abrir()
    try:
        valor = lgpio.gpio_read(chip, numero)
    except lgpio.error as exc:
        raise OSError("leitura invalida BCM %d: %s" % (numero, exc))
    if valor not in (0, 1):
        raise OSError("leitura invalida BCM %d: %r" % (numero, valor))
    return int(valor)


def preparar_led():
    if BCM_LED in (BCM_ESQUERDA, BCM_DIREITA):
        raise OSError("O LED nao pode usar o BCM de um botao")
    lgpio = _biblioteca()
    chip = _abrir()
    try:
        lgpio.gpio_free(chip, BCM_LED)
    except Exception:
        pass
    try:
        lgpio.gpio_claim_output(chip, BCM_LED, 0)
    except lgpio.error as exc:
        raise OSError("Nao reivindiquei o LED BCM %d: %s" % (BCM_LED, exc))
    escrever_led(0)


def escrever_led(nivel):
    lgpio = _biblioteca()
    chip = _abrir()
    try:
        lgpio.gpio_write(chip, BCM_LED, 1 if nivel else 0)
    except lgpio.error as exc:
        raise OSError("Nao escrevi o LED BCM %d: %s" % (BCM_LED, exc))


def linha(estado):
    partes = []
    for (nome, _numero), valor in zip(PINOS, estado):
        partes.append("%s=%d" % (nome, valor))
    return "  ".join(partes)


def imprimir(estado):
    agora = time.time()
    texto = time.strftime("%H:%M:%S", time.localtime(agora))
    print("%s.%03d  %s" % (texto, int(agora * 1000) % 1000, linha(estado)), flush=True)


def checar_logica():
    exigir(CHIP_PI4 == 0, "A Pi 4 deixou de usar o gpiochip0")
    exigir(BCM_ESQUERDA == 6 and BCM_DIREITA == 13, "O BCM dos botoes mudou")
    exigir(BCM_LED == 4, "O BCM do LED mudou")
    exigir(BCM_LED not in (BCM_ESQUERDA, BCM_DIREITA), "O LED caiu num botao")
    exigir(math_ok(), "O filtro da Pi 4 nao segue o tempo da Jetson")
    print("Logica do GPIO da Pi 4 ok. O chip nao foi aberto.", flush=True)


def math_ok():
    filtro = Filtro()
    if filtro.atualizar(1, 0.0) != 0:
        return False
    if filtro.atualizar(1, 0.09) != 0:
        return False
    if filtro.atualizar(1, 0.10) != 1:
        return False
    if filtro.atualizar(0, 0.12) != 1:
        return False
    if filtro.atualizar(0, 0.15) != 0:
        return False
    return True


def main():
    if "--checar" in sys.argv:
        checar_logica()
        return 0
    for _nome, numero in PINOS:
        exportar_entrada(numero)
    filtros = [Filtro() for _nome in PINOS]
    print(
        "Raspberry Pi 4. Pino 31 = BCM %d, pino 33 = BCM %d, pull-down.\n"
        "0 em repouso, 1 com 3,3 V. Sobe apos 100 ms, volta a 0 apos 30 ms. Ctrl+C encerra."
        % (BCM_ESQUERDA, BCM_DIREITA),
        flush=True)
    estado = (0, 0)
    proxima_tela = 0.0
    try:
        while True:
            agora = time.time()
            try:
                bruto = [ler_bruto(numero) for _nome, numero in PINOS]
            except (IOError, OSError) as exc:
                sys.stderr.write("falha de leitura, forco 0: %s\n" % exc)
                estado = (0, 0)
                imprimir(estado)
                return 1
            novo = tuple(
                filtro.atualizar(valor, agora) for filtro, valor in zip(filtros, bruto))
            if novo != estado or agora >= proxima_tela:
                if novo != estado:
                    print("mudou", flush=True)
                estado = novo
                imprimir(estado)
                proxima_tela = agora + PERIODO_TELA_S
            time.sleep(AMOSTRA_S)
    except KeyboardInterrupt:
        print("encerrado", flush=True)
        return 0


if __name__ == "__main__":
    sys.exit(main())
