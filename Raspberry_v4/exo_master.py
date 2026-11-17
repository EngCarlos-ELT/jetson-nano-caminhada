#!/usr/bin/env python3
"""Mestre do exo na Raspberry Pi 4. Entra na caminhada por botao.

Com a Pi no ar o programa entra direto no passo por botao. O LED verde
do pino 7 acende quando os motores ja seguram a pose. Os pinos 31
(BCM 6) e 33 (BCM 13) continuam entrada, com o mesmo filtro.

Segurar os dois botoes por 5 s, durante o passo, solta o torque, apaga o
LED e desliga a Pi. Enquanto os dois seguem juntos o exo nao anda.

Um aperto preso desde a partida nao conta no passo: os pinos precisam ser
vistos em 0 por 30 ms antes de armar. Falha de leitura vale 0 e nao
desliga a Pi.
"""

import os
import signal
import subprocess
import sys
import threading
import time

import gpio_raspberry as gpio
import passo_por_botao as passo


PISCA_S = 0.8
DESLIGAR_S = 5.0
AMOSTRA_S = gpio.AMOSTRA_S


def exigir(condicao, mensagem):
    if not condicao:
        raise RuntimeError(mensagem)


class PedidoDesligamento(object):
    """Conta 5 s dos dois botoes enquanto a caminhada ja esta no ar."""

    def __init__(self, desde):
        self.desde = desde
        self.desligar = False

    def observar(self, esquerda, direita, agora):
        if self.desligar:
            return True
        if esquerda != 1 or direita != 1:
            self.desde = None
            return False
        if self.desde is None:
            self.desde = agora
        if agora - self.desde >= DESLIGAR_S:
            self.desligar = True
            return True
        return False


def checar_logica():
    pedido = PedidoDesligamento(0.0)
    exigir(not pedido.observar(1, 1, 4.9), "4,9 s ja desligou")
    exigir(pedido.observar(1, 1, 5.0), "5 s nao desligou")
    exigir(pedido.desligar, "O pedido de desligamento nao ficou marcado")

    novo = PedidoDesligamento(None)
    exigir(not novo.observar(1, 0, 4.0), "Um botao sozinho desligou durante o passo")
    exigir(not novo.observar(1, 1, 8.9), "A contagem velha sobreviveu ao botao solto")
    exigir(novo.observar(1, 1, 13.9), "Um novo aperto de 5 s nao desligou")
    print("Logica do mestre ok. CAN nao foi aberta.", flush=True)


def desligar_ubuntu():
    """Pede o desligamento. systemctl so volta se o pedido foi recusado."""
    print("Desligando a Raspberry Pi.", flush=True)
    comandos = (
        ["systemctl", "poweroff"],
        ["sudo", "-n", "systemctl", "poweroff"],
        ["sudo", "-n", "shutdown", "-h", "now"],
    )
    for comando in comandos:
        try:
            if subprocess.call(comando) == 0:
                return True
        except OSError as exc:
            print("AVISO: %s falhou: %s" % (comando[0], exc), flush=True)
    print(
        "AVISO: nao consegui desligar a Raspberry Pi sem senha.\n"
        "sudo systemctl poweroff",
        flush=True)
    return False


def apagar_de_vez(led):
    led.apagar()
    if led.disponivel:
        try:
            led._escrever(0)
        except (IOError, OSError) as exc:
            print("AVISO: LED nao ficou em 0: %s" % exc, flush=True)
    print("LED verde apagado.", flush=True)


def esperar_desligar(led):
    """LED pisca. Os dois botoes por 5 s desligam. Nao religa a caminhada."""
    pedido = PedidoDesligamento(None)
    filtros = passo.preparar_botoes()
    proxima = time.monotonic()
    nivel = 0
    print(
        "Caminhada nao ficou no ar. LED piscando.\n"
        "Ajuste a pose e reinicie o mestre para tentar de novo.\n"
        "Segure os dois botoes por 5 s para desligar a Pi. Ctrl+C sai.",
        flush=True)
    while True:
        agora = time.monotonic()
        if agora >= proxima:
            nivel = 0 if nivel == 1 else 1
            led.saida(nivel)
            proxima = agora + PISCA_S
        estavel, _bruto, leitura_ok, _erro = passo.ler_botoes(filtros, agora)
        if leitura_ok and pedido.observar(
                estavel["esquerda"], estavel["direita"], agora):
            return
        falta = proxima - time.monotonic()
        time.sleep(AMOSTRA_S if falta > AMOSTRA_S else max(0.0, falta))


def vigiar_botoes(pedido, parar):
    """Le os pinos enquanto a caminhada roda. Aos 5 s pede o desligamento."""
    filtros = {nome: gpio.Filtro() for nome, _pino, _numero in passo.ENTRADAS}
    agora = time.monotonic()
    for filtro in filtros.values():
        filtro.estavel = 1
        filtro.bruto = 1
        filtro.desde = agora
    while not parar.is_set() and not pedido.desligar:
        agora = time.monotonic()
        estavel, _bruto, leitura_ok, _erro = passo.ler_botoes(filtros, agora)
        if not leitura_ok:
            pedido.desde = None
        elif pedido.observar(estavel["esquerda"], estavel["direita"], agora):
            print(
                "Os dois botoes ficaram apertados por 5 s. Vou soltar o torque e desligar.",
                flush=True)
            os.kill(os.getpid(), signal.SIGINT)
            return
        parar.wait(AMOSTRA_S)


def servir():
    led = passo.LedPronto()
    led.preparar()
    passo.preparar_botoes()
    try:
        return _servir(led)
    except KeyboardInterrupt:
        apagar_de_vez(led)
        print("Mestre encerrado. A Raspberry Pi continua ligada.", flush=True)
        return 0


def _servir(led):
    print(
        "MESTRE DO EXO\n"
        "Caminhada por botao. Entra direto, sem gesto dos dois botoes.\n"
        "O LED acende quando o exo estiver em pe.\n"
        "Segure os dois botoes por 5 s para soltar o torque e desligar a Pi.\n"
        "Ctrl+C sai sem desligar.",
        flush=True)
    while True:
        pedido = PedidoDesligamento(None)
        parar = threading.Event()
        vigia = threading.Thread(target=vigiar_botoes, args=(pedido, parar))
        vigia.daemon = True
        vigia.start()
        try:
            parada = passo.executar(led=led, pedido=pedido)
        finally:
            parar.set()
            vigia.join(timeout=1.0)
        if pedido.desligar:
            apagar_de_vez(led)
            desligar_ubuntu()
            return 0
        if parada:
            apagar_de_vez(led)
            print("Mestre encerrado. A Raspberry Pi continua ligada.", flush=True)
            return 0
        esperar_desligar(led)
        apagar_de_vez(led)
        desligar_ubuntu()
        return 0


def main():
    if "--checar" in sys.argv:
        checar_logica()
        return 0
    return servir()


if __name__ == "__main__":
    sys.exit(main())
