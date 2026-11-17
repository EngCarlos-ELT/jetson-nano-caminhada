#!/usr/bin/env python3
"""Mostra o angulo absoluto dos 4 motores, sem ligar o torque.

Serve para olhar os batentes com as pernas soltas. O unico quadro enviado
e o FD, que pede o estado e deixa o motor desligado. Nao rode junto com
exo_master.py: os dois usam a mesma CANable.
"""

import math
import sys
import time
import tkinter as tk
import tkinter.font as tkfont

import suporte_marcha as marcha


PORTA = "/dev/ttyACM0"
FD = bytes([255] * 7 + [253])
PERIODO_MS = 100
NOMES = (
    (1, "Quadril esquerdo"),
    (2, "Joelho esquerdo"),
    (3, "Quadril direito"),
    (4, "Joelho direito"),
)


def abrir(porta):
    link = marcha.Link(porta)
    link.serial.open()
    for comando in (b"C\r", b"M0\r", b"A1\r", b"S8\r", b"O\r"):
        link.write(comando)
        time.sleep(0.06)
        try:
            link.poll()
        except RuntimeError:
            link.buffer[:] = link.buffer.replace(b"\x07", b"")
    link.serial.reset_input_buffer()
    link.buffer.clear()
    return link


def pedir_estado(link):
    for joint in marcha.JOINTS:
        link.write(marcha.frame(joint.can_id, FD))
    prazo = time.monotonic() + 0.06
    while time.monotonic() < prazo:
        try:
            link.poll()
        except RuntimeError:
            link.buffer[:] = link.buffer.replace(b"\x07", b"")
        time.sleep(0.002)


def graus_de(estado):
    if estado is None:
        return None
    graus = math.degrees(estado["position_rad"])
    if not math.isfinite(graus):
        return None
    return graus


class Painel(object):
    def __init__(self, raiz, link):
        self.raiz = raiz
        self.link = link
        self.extremos = {can_id: [None, None] for can_id, _nome in NOMES}
        self.ultimo_texto = 0.0
        self.fechando = False
        self.linhas = {}
        fonte_nome = tkfont.Font(family="DejaVu Sans", size=18)
        fonte_angulo = tkfont.Font(family="DejaVu Sans", size=42, weight="bold")
        fonte_miuda = tkfont.Font(family="DejaVu Sans", size=14)
        raiz.title("Angulos dos motores — torque desligado")
        raiz.configure(bg="#101410")
        raiz.geometry("980x760")
        aviso = tk.Label(
            raiz, bg="#101410", fg="#f0d060",
            font=fonte_miuda, justify="left",
            text="Torque desligado. Apoie o exo e mova devagar ate cada batente.\n"
                 "Menor e maior sao os extremos desde a abertura desta janela.")
        aviso.pack(fill="x", padx=16, pady=(12, 4))
        for can_id, nome in NOMES:
            baixo, alto = marcha.MEASURED_BATENT_LIMITS_DEG[can_id]
            quadro = tk.Frame(raiz, bg="#1c241c", highlightthickness=1,
                               highlightbackground="#345034")
            quadro.pack(fill="x", padx=16, pady=6)
            tk.Label(
                quadro, bg="#1c241c", fg="#d7e6d7", font=fonte_nome, anchor="w",
                text="ID%d  %s" % (can_id, nome)).pack(fill="x", padx=12, pady=(8, 0))
            angulo = tk.Label(
                quadro, bg="#1c241c", fg="#b6ff9a", font=fonte_angulo, anchor="w",
                text="--")
            angulo.pack(fill="x", padx=12)
            extra = tk.Label(
                quadro, bg="#1c241c", fg="#c5d0c5", font=fonte_miuda,
                anchor="w", justify="left",
                text="batente gravado  %+.2f° a %+.2f°" % (baixo, alto))
            extra.pack(fill="x", padx=12, pady=(0, 8))
            self.linhas[can_id] = (angulo, extra, baixo, alto)
        self.estado = tk.Label(
            raiz, bg="#101410", fg="#9aa89a", font=fonte_miuda, anchor="w")
        self.estado.pack(fill="x", padx=16, pady=8)
        botoes = tk.Frame(raiz, bg="#101410")
        botoes.pack(fill="x", padx=16, pady=(0, 12))
        tk.Button(
            botoes, text="Zerar menor e maior", command=self.zerar,
            font=fonte_miuda).pack(side="left")
        tk.Button(
            botoes, text="Fechar e soltar", command=self.fechar,
            font=fonte_miuda).pack(side="right")
        raiz.protocol("WM_DELETE_WINDOW", self.fechar)
        self.atualizar()

    def zerar(self):
        for can_id in self.extremos:
            self.extremos[can_id] = [None, None]

    def fechar(self):
        if self.fechando:
            return
        self.fechando = True
        try:
            self.link.shutdown()
        except Exception as exc:
            print("AVISO ao soltar: %s" % exc, flush=True)
        self.raiz.destroy()

    def atualizar(self):
        if self.fechando:
            return
        try:
            pedir_estado(self.link)
        except Exception as exc:
            self.estado.configure(text="Falha de leitura: %s" % exc)
            self.raiz.after(PERIODO_MS, self.atualizar)
            return
        agora = time.monotonic()
        partes = []
        for can_id, _nome in NOMES:
            angulo, extra, baixo, alto = self.linhas[can_id]
            estado = self.link.states.get(can_id)
            graus = graus_de(estado)
            fresco = (
                estado is not None
                and agora - estado.get("received_at", 0.0) < 0.5
                and graus is not None)
            if not fresco:
                angulo.configure(text="sem resposta", fg="#ff8d7a")
                partes.append("ID%d=?" % can_id)
                continue
            menor, maior = self.extremos[can_id]
            menor = graus if menor is None else min(menor, graus)
            maior = graus if maior is None else max(maior, graus)
            self.extremos[can_id] = [menor, maior]
            angulo.configure(text="%+.2f°" % graus, fg="#b6ff9a")
            extra.configure(
                text="menor %+.2f°    maior %+.2f°    esforco %+.1f Nm\n"
                     "batente gravado  %+.2f° a %+.2f°"
                     % (menor, maior, estado["effort_nm"], baixo, alto))
            partes.append("ID%d=%+.2f" % (can_id, graus))
        if agora - self.ultimo_texto >= 0.5:
            self.ultimo_texto = agora
            print(" ".join(partes), flush=True)
        self.estado.configure(text="Torque desligado. Atualizando 10 vezes por segundo.")
        self.raiz.after(PERIODO_MS, self.atualizar)


def main():
    porta = sys.argv[1] if len(sys.argv) > 1 else PORTA
    link = abrir(porta)
    try:
        pedir_estado(link)
    except Exception:
        link.shutdown()
        raise
    raiz = tk.Tk()
    Painel(raiz, link)
    raiz.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
