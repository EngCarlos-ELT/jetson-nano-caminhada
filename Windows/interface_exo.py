#!/usr/bin/env python3
"""Janela do exo no Windows. A CANable v2 abre em COM6. O GPIO fica desligado.

Segure Esquerda ou Direita para o passo. Segure Em pe, ou os dois juntos,
e a referencia fica em pe sem caminhar. Soltar torque encerra os motores.
"""

import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox

import gpio_windows as gpio
import passo_por_botao as passo


PORTA_PADRAO = "COM6"
FUNDO = "#16191e"
CARTAO = "#222831"
TEXTO = "#e8eaed"
MUDO = "#9aa3ad"
VERDE = "#2f9e6b"
AMBAR = "#e0a045"
VERMELHO = "#c4564a"
AZUL = "#3d7ea6"
BOTAO = "#2c3542"
NOMES_JUNTAS = {
    1: "Quadril esquerdo",
    2: "Joelho esquerdo",
    3: "Quadril direito",
    4: "Joelho direito",
}


def exigir(condicao, mensagem):
    if not condicao:
        raise RuntimeError(mensagem)


def niveis_comandados(mouse, teclas, pronto):
    """1 enquanto o botao ou a tecla correspondente fica apertado."""
    if not pronto:
        return 0, 0
    if mouse == "em_pe" or "space" in teclas:
        if getattr(passo, "AMBOS_COMANDOS_ANDAM", False):
            return 0, 0
        return 1, 1
    esquerda = mouse == "esquerda" or "Left" in teclas or "a" in teclas or "A" in teclas
    direita = mouse == "direita" or "Right" in teclas or "d" in teclas or "D" in teclas
    return (1 if esquerda else 0), (1 if direita else 0)


def listar_portas():
    portas = []
    try:
        from serial.tools import list_ports
        portas = [item.device for item in list_ports.comports()]
    except Exception:
        portas = []
    if PORTA_PADRAO not in portas:
        portas.insert(0, PORTA_PADRAO)
    return portas


class PedidoTorque(object):
    def __init__(self):
        self.desligar = False


class Janela(object):
    def __init__(self, porta=None):
        self.estado = gpio.Estado()
        self.pedido = PedidoTorque()
        self.thread = None
        self.mouse = None
        self.teclas = set()
        self.pronto = False
        self.root = tk.Tk()
        self.root.title(getattr(passo, "TITULO_JANELA", "Exo — Windows"))
        self.root.configure(bg=FUNDO)
        self.root.geometry("980x740")
        self.root.minsize(860, 660)
        self._montar(porta or PORTA_PADRAO)
        self.root.protocol("WM_DELETE_WINDOW", self.fechar)
        self.root.bind("<KeyPress>", self.tecla_desce)
        self.root.bind("<KeyRelease>", self.tecla_sobe)
        self.root.bind("<ButtonRelease-1>", self.mouse_solta)
        self.root.after(80, self.atualizar)

    def _montar(self, porta):
        topo = tk.Frame(self.root, bg=FUNDO)
        topo.pack(fill="x", padx=22, pady=(18, 8))
        tk.Label(
            topo, text="Exo", bg=FUNDO, fg=TEXTO,
            font=("Segoe UI", 26, "bold")).pack(side="left")
        tk.Label(
            topo, text="CANable v2  ·  GPIO desligado",
            bg=FUNDO, fg=MUDO, font=("Segoe UI", 12)).pack(side="left", padx=16, pady=(10, 0))

        barra = tk.Frame(self.root, bg=CARTAO)
        barra.pack(fill="x", padx=22, pady=6)
        tk.Label(barra, text="Porta", bg=CARTAO, fg=MUDO,
                 font=("Segoe UI", 11)).pack(side="left", padx=(14, 8), pady=12)
        self.porta = tk.StringVar(value=porta)
        self.combo = tk.Entry(
            barra, textvariable=self.porta, width=12, bg="#12151a", fg=TEXTO,
            insertbackground=TEXTO, relief="flat", font=("Segoe UI", 14))
        self.combo.pack(side="left", pady=12)
        self.botao_portas = self._botao(barra, "Atualizar", self.atualizar_portas, BOTAO)
        self.botao_portas.pack(side="left", padx=8)
        self.botao_ligar = self._botao(barra, "Ligar", self.ligar, VERDE)
        self.botao_ligar.pack(side="left", padx=(18, 8))
        self.botao_soltar = self._botao(barra, "Soltar torque", self.soltar_torque, VERMELHO)
        self.botao_soltar.pack(side="left", padx=8)
        self.botao_soltar.configure(state="disabled")

        estado = tk.Frame(self.root, bg=CARTAO)
        estado.pack(fill="x", padx=22, pady=10)
        linha = tk.Frame(estado, bg=CARTAO)
        linha.pack(fill="x", padx=16, pady=(14, 0))
        self.luz = tk.Canvas(linha, width=22, height=22, bg=CARTAO, highlightthickness=0)
        self.luz.pack(side="left")
        self.lampada = self.luz.create_oval(3, 3, 19, 19, fill="#3a414c", outline="")
        self.rotulo_fase = tk.Label(
            linha, text="Aguardando", bg=CARTAO, fg=TEXTO,
            font=("Segoe UI", 18, "bold"))
        self.rotulo_fase.pack(side="left", padx=10)
        self.rotulo_titulo = tk.Label(
            estado, text=self.estado.titulo, bg=CARTAO, fg=MUDO,
            font=("Segoe UI", 12), anchor="w", justify="left")
        self.rotulo_titulo.pack(fill="x", padx=16, pady=(4, 0))
        self.rotulo_botoes = tk.Label(
            estado, text="Esquerda livre  |  Direita livre", bg=CARTAO, fg=TEXTO,
            font=("Segoe UI", 12), anchor="w")
        self.rotulo_botoes.pack(fill="x", padx=16, pady=(2, 14))

        comandos = tk.Frame(self.root, bg=FUNDO)
        comandos.pack(fill="x", padx=22, pady=6)
        comandos.columnconfigure(0, weight=1)
        comandos.columnconfigure(1, weight=1)
        comandos.columnconfigure(2, weight=1)
        self.cmd_esquerda = self._comando(comandos, "ESQUERDA", "segure  ·  seta ←", "esquerda", 0)
        self.cmd_empe = self._comando(comandos, "EM PÉ", "segure  ·  espaço", "em_pe", 1)
        self.cmd_direita = self._comando(comandos, "DIREITA", "segure  ·  seta →", "direita", 2)

        juntas = tk.Frame(self.root, bg=FUNDO)
        juntas.pack(fill="x", padx=22, pady=8)
        for coluna in range(4):
            juntas.columnconfigure(coluna, weight=1)
        self.juntas = {}
        for indice, can_id in enumerate((1, 2, 3, 4)):
            cartao = tk.Frame(juntas, bg=CARTAO)
            cartao.grid(row=0, column=indice, sticky="nsew", padx=4)
            tk.Label(
                cartao, text=NOMES_JUNTAS[can_id], bg=CARTAO, fg=MUDO,
                font=("Segoe UI", 10)).pack(anchor="w", padx=12, pady=(10, 0))
            graus = tk.Label(
                cartao, text="—", bg=CARTAO, fg=TEXTO, font=("Segoe UI", 20, "bold"))
            graus.pack(anchor="w", padx=12)
            esforco = tk.Label(
                cartao, text="", bg=CARTAO, fg=MUDO, font=("Segoe UI", 11))
            esforco.pack(anchor="w", padx=12, pady=(0, 10))
            self.juntas[can_id] = (graus, esforco)

        tk.Label(
            self.root, text="Eventos", bg=FUNDO, fg=MUDO,
            font=("Segoe UI", 11)).pack(anchor="w", padx=26)
        self.log = tk.Text(
            self.root, height=7, bg="#12151a", fg=TEXTO, relief="flat",
            font=("Segoe UI", 11), wrap="word", padx=10, pady=8)
        self.log.pack(fill="both", expand=True, padx=22, pady=(2, 8))
        self.log.configure(state="disabled")
        tk.Label(
            self.root,
            text=getattr(
                passo, "DESCRICAO_JANELA",
                "Mantenha o exo apoiado. Soltar antes de 70% pausa o passo. "
                "Depois de 70% ele termina sozinho."),
            bg=FUNDO, fg=MUDO, font=("Segoe UI", 10)).pack(anchor="w", padx=26, pady=(0, 14))

    def _botao(self, pai, texto, comando, cor):
        return tk.Button(
            pai, text=texto, command=comando, bg=cor, fg="white",
            activebackground=cor, activeforeground="white",
            relief="flat", bd=0, padx=16, pady=8,
            font=("Segoe UI", 12, "bold"), cursor="hand2")

    def _comando(self, pai, titulo, dica, nome, coluna):
        quadro = tk.Frame(pai, bg=BOTAO)
        quadro.grid(row=0, column=coluna, sticky="nsew", padx=4)
        rotulo = tk.Label(
            quadro, text=titulo, bg=BOTAO, fg="white",
            font=("Segoe UI", 20, "bold"), pady=18)
        rotulo.pack(fill="x")
        sub = tk.Label(quadro, text=dica, bg=BOTAO, fg=MUDO, font=("Segoe UI", 10))
        sub.pack(pady=(0, 14))
        for peca in (quadro, rotulo, sub):
            peca.bind("<ButtonPress-1>", lambda _evento, qual=nome: self.mouse_aperta(qual))
        return quadro

    def atualizar_portas(self):
        portas = listar_portas()
        atual = self.porta.get().strip()
        if atual and atual not in portas:
            portas.insert(0, atual)
        self.anotar("Portas vistas: %s" % ", ".join(portas))

    def ligar(self):
        if self.thread is not None and self.thread.is_alive():
            return
        porta = self.porta.get().strip() or PORTA_PADRAO
        self.porta.set(porta)
        self.pedido = PedidoTorque()
        gpio.COMANDOS.soltar()
        self.estado.definir(
            rodando=True, pronto=False, fase="ligando", erro="",
            titulo="Abrindo %s. GPIO desligado." % porta, eventos=[], juntas=[])
        self.anotar("Ligando %s." % porta)
        self.thread = threading.Thread(
            target=self._rodar, args=(porta, self.pedido, self.estado),
            name="exo-can")
        self.thread.start()

    def _rodar(self, porta, pedido, estado):
        try:
            passo.executar(porta=porta, pedido=pedido, estado=estado, sinais=False)
        except Exception as exc:
            estado.definir(
                fase="erro", erro=str(exc), titulo="Falhou: %s" % exc,
                rodando=False, pronto=False)

    def soltar_torque(self):
        if self.thread is None or not self.thread.is_alive():
            return
        gpio.COMANDOS.soltar()
        self.mouse = None
        self.teclas.clear()
        self.pedido.desligar = True
        self.anotar("Soltando o torque.")

    def mouse_aperta(self, nome):
        self.mouse = nome
        self.aplicar()

    def mouse_solta(self, _evento=None):
        if self.mouse is None:
            return
        self.mouse = None
        self.aplicar()

    def tecla_desce(self, evento):
        if evento.widget is self.combo:
            return
        if evento.keysym in ("Left", "Right", "space", "a", "A", "d", "D"):
            self.teclas.add(evento.keysym)
            self.aplicar()

    def tecla_sobe(self, evento):
        self.teclas.discard(evento.keysym)
        self.aplicar()

    def aplicar(self):
        esquerda, direita = niveis_comandados(self.mouse, self.teclas, self.pronto)
        gpio.COMANDOS.definir("esquerda", esquerda)
        gpio.COMANDOS.definir("direita", direita)
        self._pintar_comandos(esquerda, direita)

    def _pintar_comandos(self, esquerda, direita):
        if getattr(passo, "AMBOS_COMANDOS_ANDAM", False):
            empe = self.pronto and (self.mouse == "em_pe" or "space" in self.teclas)
            pares = (
                (self.cmd_esquerda, bool(esquerda)),
                (self.cmd_direita, bool(direita)),
                (self.cmd_empe, empe),
            )
        else:
            pares = (
                (self.cmd_esquerda, esquerda and not direita),
                (self.cmd_direita, direita and not esquerda),
                (self.cmd_empe, esquerda and direita),
            )
        for quadro, aceso in pares:
            cor = AMBAR if aceso else BOTAO
            if quadro is self.cmd_empe and aceso:
                cor = VERDE
            self._pintar_quadro(quadro, cor)

    def _pintar_quadro(self, quadro, cor):
        quadro.configure(bg=cor)
        for filho in quadro.winfo_children():
            filho.configure(bg=cor)

    def atualizar(self):
        foto = self.estado.foto()
        self.pronto = bool(foto["pronto"])
        if not self.pronto and (self.mouse or self.teclas):
            self.mouse = None
            self.teclas.clear()
            gpio.COMANDOS.soltar()
        self.rotulo_fase.configure(text=self._fase(foto["fase"]))
        self.rotulo_titulo.configure(text=foto["titulo"])
        if foto["botoes"]:
            self.rotulo_botoes.configure(text=foto["botoes"])
        self._lampada(foto)
        self._juntas(foto["juntas"])
        self._log(foto["eventos"])
        rodando = self.thread is not None and self.thread.is_alive()
        self.botao_ligar.configure(state="disabled" if rodando else "normal")
        self.botao_soltar.configure(state="normal" if rodando else "disabled")
        self.combo.configure(state="disabled" if rodando else "normal")
        self.aplicar()
        self.root.after(80, self.atualizar)

    def _fase(self, fase):
        return {
            "parado": "Aguardando",
            "ligando": "Ligando",
            "em_pe": "Em pé",
            "passo": "Passo",
            "pausa": "Pausa",
            "erro": "Interrompido",
        }.get(fase, fase)

    def _lampada(self, foto):
        if foto["fase"] == "erro":
            cor = VERMELHO
        elif foto["pronto"] and foto["led"]:
            cor = VERDE
        elif foto["rodando"] and int(time.monotonic() * 2) % 2 == 0:
            cor = AMBAR
        else:
            cor = "#3a414c"
        self.luz.itemconfigure(self.lampada, fill=cor)

    def _juntas(self, juntas):
        por_id = {item["id"]: item for item in juntas}
        for can_id, (graus, esforco) in self.juntas.items():
            item = por_id.get(can_id)
            if item is None:
                graus.configure(text="—")
                esforco.configure(text="")
                continue
            graus.configure(text="%+.1f°" % item["graus"])
            esforco.configure(text="%+.1f Nm   %.0f °C" % (
                item["esforco_nm"], item["temperatura_c"]))

    def _log(self, eventos):
        texto = "\n".join(eventos)
        if texto == getattr(self, "_log_visto", None):
            return
        self._log_visto = texto
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        if texto:
            self.log.insert("end", texto + "\n")
        self.log.configure(state="disabled")
        self.log.see("end")

    def anotar(self, texto):
        self.estado.evento(texto)

    def fechar(self):
        vivo = self.thread is not None and self.thread.is_alive()
        if vivo:
            seguir = messagebox.askokcancel(
                "Exo",
                "Soltar o torque e fechar a janela?")
            if not seguir:
                return
            self.soltar_torque()
            self.thread.join(8)
        self.root.destroy()

    def executar(self):
        self.root.mainloop()


def checar_logica():
    gpio.checar_logica()
    exigir(niveis_comandados(None, set(), False) == (0, 0),
           "A janela andou antes de o exo ficar pronto")
    exigir(niveis_comandados("esquerda", set(), True) == (1, 0),
           "Esquerda nao segurou o comando")
    exigir(niveis_comandados("direita", set(), True) == (0, 1),
           "Direita nao segurou o comando")
    exigir(niveis_comandados(None, {"Left", "Right"}, True) == (1, 1),
           "As duas setas nao pediram em pe")
    exigir(niveis_comandados("em_pe", set(), True) == (1, 1),
           "Em pe nao segurou os dois comandos")
    exigir(niveis_comandados(None, set(), True) == (0, 0),
           "Soltar a janela deixou um comando apertado")
    exigir(passo.GPIO_ATIVO is False, "O passo Windows ligou o GPIO")
    exigir(passo.caminhada.PORTA_PADRAO == "COM6", "A porta padrao deixou de ser COM6")
    exigir(passo.caminhada.resolver_porta(None) == "COM6", "Sem argumento a CAN nao ficou em COM6")
    exigir(passo.caminhada.resolver_porta("com5") == "COM5", "A COM pedida nao foi aceita")
    print("Logica da janela ok. CAN nao foi aberta.", flush=True)


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    if "--checar" in argv:
        checar_logica()
        return 0
    porta = None
    for item in argv:
        if not item.startswith("-"):
            porta = item
            break
    try:
        if sys.platform == "win32":
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    janela = Janela(porta)
    janela.executar()
    return 0


if __name__ == "__main__":
    sys.exit(main())
