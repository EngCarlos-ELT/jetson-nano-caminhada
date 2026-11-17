#!/usr/bin/env python3
"""Le os pinos 31 (D6) e 33 (D13) com nivel seguro.

Cada pino tem um resistor de 1 kΩ para o GND do header.
0 = so o resistor, sem 3,3 V.
1 = 3,3 V firme no pino, vencendo o resistor.
Um pico so vira 1 depois de 100 ms estaveis. A volta para 0 confirma em 30 ms,
para o pino nao permanecer armado depois que o 3,3 V sai.
Se o pinmux deste boot nao foi configurado, o programa para em vez de
publicar um nivel falso.
"""

import os
import subprocess
import sys
import time

SYSFS = "/sys/class/gpio"
MARCA = "/run/gpio-31-33-entrada.ok"
PASTA = os.path.dirname(os.path.abspath(__file__))
CONFIGURADOR = os.path.join(PASTA, "configurar_gpios_31_33.py")
AMOSTRA_S = 0.01
CONFIRMA_UM_S = 0.10
CONFIRMA_ZERO_S = 0.03
PERIODO_TELA_S = 0.2
PINOS = (
    ("pino31_D6", 200),
    ("pino33_D13", 38),
)


def pinmux_deste_boot():
    try:
        with open(MARCA, "r") as arquivo:
            gravado = arquivo.read().strip()
        with open("/proc/sys/kernel/random/boot_id", "r") as arquivo:
            atual = arquivo.read().strip()
    except (IOError, OSError):
        return False
    return bool(gravado) and gravado == atual


def configurar():
    if os.geteuid() != 0:
        return False
    return subprocess.call([sys.executable, CONFIGURADOR]) == 0 and pinmux_deste_boot()


def exportar_entrada(numero):
    pasta = os.path.join(SYSFS, "gpio%d" % numero)
    if not os.path.isdir(pasta):
        with open(os.path.join(SYSFS, "export"), "w") as arquivo:
            arquivo.write("%d" % numero)
    direcao = os.path.join(pasta, "direction")
    prazo = time.time() + 2.0
    while True:
        try:
            with open(direcao, "w") as arquivo:
                arquivo.write("in")
            return
        except (IOError, OSError):
            if time.time() > prazo:
                raise
            time.sleep(0.05)


def ler_bruto(numero):
    with open(os.path.join(SYSFS, "gpio%d" % numero, "value"), "r") as arquivo:
        texto = arquivo.read().strip()
    if texto not in ("0", "1"):
        raise IOError("leitura invalida gpio %d: %r" % (numero, texto))
    return int(texto)


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


def linha(estado):
    partes = []
    for (nome, _numero), valor in zip(PINOS, estado):
        partes.append("%s=%d" % (nome, valor))
    return "  ".join(partes)


def imprimir(estado):
    agora = time.time()
    texto = time.strftime("%H:%M:%S", time.localtime(agora))
    print("%s.%03d  %s" % (texto, int(agora * 1000) % 1000, linha(estado)), flush=True)


def main():
    if not pinmux_deste_boot() and not configurar():
        sys.stderr.write(
            "AVISO: pinmux deste boot nao confirmado. A leitura segue.\n"
            "sudo python3 configurar_gpios_31_33.py\n"
        )
    for _nome, numero in PINOS:
        exportar_entrada(numero)
    filtros = [Filtro() for _nome in PINOS]
    print(
        "Pinos 31 e 33 com 1 kΩ para o terra: 0 em repouso, 1 com 3,3 V. "
        "Sobe apos 100 ms, volta a 0 apos 30 ms. Ctrl+C encerra.",
        flush=True,
    )
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
                filtro.atualizar(valor, agora) for filtro, valor in zip(filtros, bruto)
            )
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
