#!/usr/bin/env python3
"""Deixa os pinos 31 e 33 como entrada segura na Jetson Nano.

Pino 31 = GPIO Linux 200 (PZ0). Pino 33 = GPIO Linux 38 (PE6).
O pad fica em alta impedancia, com receptor ligado, pull-down e Schmitt.
O nivel 0 de repouso vem do resistor de 1 kΩ entre o pino e o GND.
Fonte de 3,3 V le 1. Nao comanda os pinos como saida.
Exige root por causa do registrador do pinmux. Vale ate o proximo reboot;
o servico de boot repete esta mesma configuracao.
"""

import mmap
import os
import struct
import sys
import time

SYSFS = "/sys/class/gpio"
BASE = 0x70003000
PAGINA = 4096
MARCA = "/run/gpio-31-33-entrada.ok"
# Offset dentro de PINMUX_AUX (0x70003000), do pinctrl Tegra210.
PINOS = (
    ("pino31_D6", 200, 0x27C),
    ("pino33_D13", 38, 0x248),
)
PUPD_MASK = 0x3 << 2
PUPD_DOWN = 0x1 << 2
TRISTATE = 1 << 4
PARK = 1 << 5
E_INPUT = 1 << 6
LOCK = 1 << 7
LPDR = 1 << 8
SCHMITT = 1 << 12
PUPD_NOME = ("nenhum", "down", "up", "reservado")


def exigir_root():
    if os.geteuid() != 0:
        sys.stderr.write("rode com root: sudo python3 configurar_gpios_31_33.py\n")
        return 1
    return 0


def descrever(valor):
    pupd = (valor >> 2) & 3
    return (
        "mux=%d pupd=%s tri=%d park=%d ein=%d lock=%d lpdr=%d schmitt=%d raw=0x%08X"
        % (
            valor & 3,
            PUPD_NOME[pupd],
            (valor >> 4) & 1,
            (valor >> 5) & 1,
            (valor >> 6) & 1,
            (valor >> 7) & 1,
            (valor >> 8) & 1,
            (valor >> 12) & 1,
            valor,
        )
    )


def entrada_segura(valor):
    return (
        (valor & PUPD_MASK) == PUPD_DOWN
        and (valor & TRISTATE)
        and (valor & E_INPUT)
        and (valor & SCHMITT)
        and not (valor & PARK)
        and not (valor & LPDR)
        and not (valor & LOCK)
    )


def ajustar(valor):
    if valor & LOCK:
        raise RuntimeError("pinmux travado, nao altero o registrador")
    if valor & 0xFFFF0000:
        raise RuntimeError("leitura implausivel 0x%08X" % valor)
    valor = (valor & ~PUPD_MASK) | PUPD_DOWN
    valor |= TRISTATE | E_INPUT | SCHMITT
    valor &= ~(PARK | LPDR)
    return valor


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


def ler_gpio(numero):
    with open(os.path.join(SYSFS, "gpio%d" % numero, "value"), "r") as arquivo:
        return int(arquivo.read().strip())


def gravar_marca():
    with open("/proc/sys/kernel/random/boot_id", "r") as arquivo:
        boot = arquivo.read().strip()
    with open(MARCA, "w") as arquivo:
        arquivo.write(boot + "\n")


def main():
    erro = exigir_root()
    if erro:
        return erro
    fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    try:
        memoria = mmap.mmap(
            fd, PAGINA, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=BASE
        )
    except (OSError, ValueError) as exc:
        os.close(fd)
        sys.stderr.write("nao abri o pinmux em 0x%08X: %s\n" % (BASE, exc))
        return 1
    try:
        for nome, numero, offset in PINOS:
            memoria.seek(offset)
            atual = struct.unpack("<I", memoria.read(4))[0]
            print("%s antes  %s" % (nome, descrever(atual)), flush=True)
            novo = ajustar(atual)
            memoria.seek(offset)
            memoria.write(struct.pack("<I", novo))
            memoria.seek(offset)
            lido = struct.unpack("<I", memoria.read(4))[0]
            print("%s depois %s" % (nome, descrever(lido)), flush=True)
            if not entrada_segura(lido):
                sys.stderr.write("%s nao confirmou a entrada com pull-down\n" % nome)
                return 1
            exportar_entrada(numero)
        time.sleep(0.02)
        altos = []
        for nome, numero, _offset in PINOS:
            valor = ler_gpio(numero)
            print("%s value=%d" % (nome, valor), flush=True)
            if valor != 0:
                altos.append(nome)
        gravar_marca()
        print("pinmux ok", flush=True)
        if altos:
            sys.stderr.write(
                "AVISO: %s leu 1. Com o resistor de 1 kΩ para o terra "
                "e sem 3,3 V no pino, o esperado e 0.\n" % " e ".join(altos)
            )
        return 0
    finally:
        memoria.close()
        os.close(fd)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (IOError, OSError, RuntimeError) as exc:
        sys.stderr.write("%s\n" % exc)
        sys.exit(1)
