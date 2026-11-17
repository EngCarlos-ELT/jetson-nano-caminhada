#!/usr/bin/env python3
"""Botoes virtuais da versao Windows. Nenhum GPIO e aberto.

A interface escreve 0 ou 1. O mesmo filtro da marcha confirma 1 em 100 ms
e 0 em 30 ms. Os dois em 1 continuam sem escolher perna.
"""

import threading


AMOSTRA_S = 0.01
CONFIRMA_UM_S = 0.10
CONFIRMA_ZERO_S = 0.03
GPIO_ATIVO = False


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


class Comandos(object):
    """Niveis brutos da janela. A marcha so le."""

    def __init__(self):
        self._trava = threading.Lock()
        self.esquerda = 0
        self.direita = 0
        self.led = 0

    def definir(self, nome, nivel):
        valor = 1 if nivel else 0
        with self._trava:
            if nome == "esquerda":
                self.esquerda = valor
            elif nome == "direita":
                self.direita = valor
            else:
                raise KeyError(nome)

    def definir_led(self, nivel):
        with self._trava:
            self.led = 1 if nivel else 0

    def ler(self, nome):
        with self._trava:
            if nome == "esquerda":
                return self.esquerda
            if nome == "direita":
                return self.direita
            if nome == "led":
                return self.led
        raise KeyError(nome)

    def soltar(self):
        with self._trava:
            self.esquerda = 0
            self.direita = 0

    def foto(self):
        with self._trava:
            return {
                "esquerda": self.esquerda,
                "direita": self.direita,
                "led": self.led,
            }


COMANDOS = Comandos()


def pinmux_deste_boot():
    return True


def exportar_entrada(_numero):
    return None


def ler_bruto(nome):
    return COMANDOS.ler(nome)


class Estado(object):
    """O que a janela mostra. A marcha publica, a interface so le."""

    def __init__(self):
        self._trava = threading.Lock()
        self.fase = "parado"
        self.titulo = "GPIO desligado. Escolha a COM e ligue."
        self.botoes = ""
        self.juntas = []
        self.eventos = []
        self.erro = ""
        self.rodando = False
        self.pronto = False

    def definir(self, **campos):
        with self._trava:
            for chave, valor in campos.items():
                setattr(self, chave, valor)

    def evento(self, texto):
        with self._trava:
            self.eventos.append(texto)
            del self.eventos[:-12]

    def foto(self):
        with self._trava:
            return {
                "fase": self.fase,
                "titulo": self.titulo,
                "botoes": self.botoes,
                "juntas": list(self.juntas),
                "eventos": list(self.eventos),
                "erro": self.erro,
                "rodando": self.rodando,
                "pronto": self.pronto,
                "led": COMANDOS.ler("led"),
                "esquerda": COMANDOS.ler("esquerda"),
                "direita": COMANDOS.ler("direita"),
            }


def checar_logica():
    if GPIO_ATIVO:
        raise RuntimeError("O GPIO do Windows ficou ativo")
    filtro = Filtro()
    if filtro.atualizar(1, 0.0) != 0 or filtro.atualizar(1, 0.10) != 1:
        raise RuntimeError("O filtro da interface nao confirma 100 ms")
    if filtro.atualizar(0, 0.12) != 1 or filtro.atualizar(0, 0.15) != 0:
        raise RuntimeError("O filtro da interface nao solta em 30 ms")
    comandos = Comandos()
    comandos.definir("esquerda", 1)
    comandos.definir("direita", 0)
    if comandos.ler("esquerda") != 1 or comandos.ler("direita") != 0:
        raise RuntimeError("O comando da janela nao chegou na marcha")
    comandos.soltar()
    if comandos.ler("esquerda") != 0 or comandos.ler("direita") != 0:
        raise RuntimeError("Soltar a janela deixou um botao apertado")
    print("Logica da interface Windows ok. GPIO nao foi aberto.", flush=True)
