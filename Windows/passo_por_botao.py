#!/usr/bin/env python3
"""Um passo por botao, separado da caminhada continua.

O exo comeca em pe. A pose lida na partida vira a origem e os motores
seguram essa pose com o mesmo torque da caminhada. A partida nao recusa
o angulo do encoder: o usuario confere a pose a olho. Sem botao valido a
referencia nao anda, entao o comando nao derruba o exo no chao.

Versao Windows, CANable v2 em COM6. O GPIO fica desligado.
A janela segura Esquerda, Direita ou Em pe no lugar dos botoes.
1 so vale depois de 100 ms firmes. A soltura confirma em 30 ms.
Uma falha de leitura vale 0. Antes de 70% da trajetoria isso congela o passo.

O passo so avanca enquanto o botao daquela perna fica em 1. Soltar antes
de 70% congela o tempo da curva e os motores seguram o angulo ja alcancado.
Depois de 70%, soltar conclui o passo normalmente. O outro botao sozinho
e ignorado ate esse passo terminar. Os dois juntos nao movem: se havia
passo, ele nao conta, a referencia volta para em pe e fica la.

No fim do passo a perna volta para a referencia em pe e o botao dela fica
desabilitado. So um aperto novo no botao da outra perna continua. Quando a
segunda perna tambem termina, o ciclo fecha, os dois botoes voltam a valer
e o exo segue em pe.

Se durante o passo o motor fica no teto de torque sem progredir, esse passo
nao conta. A referencia volta para em pe, o mesmo botao pode tentar de novo
depois de soltar, e a amplitude do proximo aperto desce. Parado ou em pausa
o teto de torque continua segurando a referencia; o programa nao troca a
pose em pe por outro angulo.

Um quadro isolado de temperatura, velocidade ou batente nao encerra: so
tres leituras seguidas. A CAN reenvia o quadro tres vezes e escuta 100 ms
em cada tentativa, inclusive quando a porta USB estoura o tempo. Um byte
de rejeicao ao lado de um quadro valido nao apaga esse quadro. Se um motor fica sem resposta por 500 ms, o ultimo comando MIT e
repetido ate 3 vezes. Na segunda, o canal CAN fecha e abre de novo.
So entao o torque e solto. Um atraso curto do ciclo nao fica acumulado
para derrubar o ciclo seguinte. A partida repete uma vez o pedido de
feedback, a leitura de esforco em repouso e o aquecimento do encoder.
Se a porta CAN estiver ocupada ou ainda nao existir, a abertura espera
ate 60 s, com o indicador piscando. Os motores ficam sem torque nessa espera.

A curva e um passo do gerador ja afinado (coxa 18°, joelho 45°, metade do
ciclo) e mexe nas duas pernas ao mesmo tempo. A perna do botao vai a frente
e a outra vai para tras, com o mesmo espelhamento dos motores da caminhada:
no quadril direito, o angulo positivo e para tras. So o joelho da perna
acionada flexiona; o da outra fica reto. O passo comeca e termina em pe.

O indicador verde fica na janela. Acende quando os motores seguram a pose.
Nenhum pino de GPIO e exportado.

Nao rode junto com caminhada_torque_pleno.py. Soltar torque desliga os motores.
python3 interface_exo.py abre a janela. --checar nao abre a CAN.
"""

import errno
import json
import math
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import caminhada_torque_pleno as caminhada
import gpio_windows as gpio
import suporte_marcha as marcha


DURACAO_PASSO_S = caminhada.DURACAO_CICLO_S / 2.0
# Sem quadro novo de um motor por este tempo, o programa solta o torque.
FEEDBACK_MAX_S = 0.5
# Falha do driver so encerra se permanecer pelo menos este tempo.
FEEDBACK_FALHA_CONTINUA_S = 0.5
# Folga alem do batente de software antes de encerrar.
FOLGA_BATENTE_GRAUS = 2.0
# Atraso do ciclo de 20 ms que ainda nao encerra.
ATRASO_LOOP_S = 0.20
# Reenvios depois de uma falha de quadro na CANable.
TENTATIVAS_EXTRAS_CAN = 3
# Um quadro isolado de velocidade, temperatura ou batente nao encerra.
LEITURAS_PARA_CONFIRMAR = 3
# Depois que a CAN volta, um pico de atraso nesse intervalo nao encerra.
JANELA_APOS_CAN_S = 1.0
# Cada reenvio espera a resposta por este tempo antes de mandar outro quadro.
PRAZO_RESPOSTA_CAN_S = 0.10
# Repetir este quadro desligaria o torque. Nunca e usado para recuperar feedback.
COMANDO_DESLIGA = b"FFFFFFFFFFFFFFFD"
# Depois desta fracao da trajetoria, soltar o botao conclui o passo.
FRACAO_PARA_CONCLUIR = 0.70
# A porta serial pode nascer ocupada. Esperar nao liga torque.
ESPERA_PORTA_S = 60.0
PAUSA_PORTA_S = 0.5
_ERRNOS_PORTA_TRANSITORIA = (
    errno.EBUSY, errno.ENOENT, errno.ENODEV, errno.EAGAIN, errno.ENXIO)
# step_count=1 no gerador afinado: a caminhada inteira dura metade do ciclo
# e as quatro juntas voltam a 0. As duas pernas andam juntas nesse trecho.
PASSOS_NO_GERADOR = 1
JUNTAS = {joint.name: joint for joint in marcha.JOINTS}
ENTRADAS = (
    ("esquerda", 0, "esquerda"),
    ("direita", 0, "direita"),
)
GPIO_ATIVO = False
LED_PINO = None
LED_GPIO = None


class LedPronto(object):
    """Indicador da janela. 0 na partida e no fim. 1 com o exo pronto."""

    def __init__(self):
        self.disponivel = False
        self.aceso = False

    def preparar(self):
        if GPIO_ATIVO:
            raise RuntimeError("O GPIO desta versao Windows nao pode ser ligado")
        self._escrever(0)
        self.disponivel = True

    def saida(self, nivel):
        if not self.disponivel or self.aceso:
            return
        self._escrever(1 if nivel else 0)

    def acender(self):
        if not self.disponivel:
            return
        self._escrever(1)
        if self.aceso:
            return
        self.aceso = True
        print("Indicador verde aceso. Exo pronto para os comandos da janela.", flush=True)

    def apagar(self):
        if not self.disponivel:
            return
        self._escrever(0)
        if self.aceso:
            print("Indicador verde apagado.", flush=True)
        self.aceso = False

    def _escrever(self, nivel):
        gpio.COMANDOS.definir_led(nivel)


def exigir(condicao, mensagem):
    if not condicao:
        raise RuntimeError(mensagem)


def porta_merece_espera(exc):
    """Porta ausente ou ocupada por outro programa. Ainda nao ha torque."""
    return getattr(exc, "errno", None) in _ERRNOS_PORTA_TRANSITORIA


def abrir_quando_livre(abrir, prazo_s=None, pausa_s=None, agora=None, dormir=None,
                       avisar=None, enquanto_espera=None):
    """Abre a porta. Se ela estiver ocupada ou ausente, espera sem ligar torque."""
    if prazo_s is None:
        prazo_s = ESPERA_PORTA_S
    if pausa_s is None:
        pausa_s = PAUSA_PORTA_S
    if agora is None:
        agora = time.monotonic
    if dormir is None:
        dormir = time.sleep
    inicio = agora()
    avisou = False
    while True:
        try:
            abrir()
        except OSError as exc:
            if not porta_merece_espera(exc):
                raise
            decorrido = agora() - inicio
            if decorrido >= prazo_s:
                raise
            if not avisou and avisar is not None:
                avisar(
                    "Porta CAN ocupada ou ausente. Esperando ate %.0f s. "
                    "Motores sem torque. %s" % (prazo_s, exc))
                avisou = True
            if enquanto_espera is not None:
                enquanto_espera()
            falta = prazo_s - decorrido
            dormir(pausa_s if falta > pausa_s else falta)
            continue
        if avisou and avisar is not None:
            avisar("Porta CAN livre.")
        return


def acompanhar_falha(falhas, can_id, codigo, agora):
    """'temporaria' no primeiro quadro, 'continua' apos o tempo, 'sumiu' ao limpar."""
    if codigo:
        desde = falhas.get(can_id)
        if desde is None:
            falhas[can_id] = agora
            return "temporaria"
        if agora - desde >= FEEDBACK_FALHA_CONTINUA_S:
            return "continua"
        return None
    if can_id in falhas:
        del falhas[can_id]
        return "sumiu"
    return None


def falha_de_comunicacao(exc):
    if isinstance(exc, OSError):
        return True
    texto = str(exc)
    return (
        "Escrita serial incompleta" in texto
        or "CANable rejeitou" in texto
        or "Buffer serial" in texto
    )


def reenviar_ate_voltar(escrever, ler, limpar, quadro, quadro_de_motor,
                        esperar=None, prazo_s=None):
    """Reenvia o quadro ate 3 vezes. Devolve o numero do reenvio que voltou, ou 0.

    Num quadro de motor, cada tentativa fica escutando ate o prazo. Uma
    resposta que chega um pouco depois da escrita ainda conta como volta.
    """
    if esperar is None:
        esperar = lambda: time.sleep(0.005)
    if prazo_s is None:
        prazo_s = PRAZO_RESPOSTA_CAN_S
    for numero in range(1, TENTATIVAS_EXTRAS_CAN + 1):
        limpar()
        try:
            escrever(quadro)
        except (RuntimeError, OSError) as exc:
            if not falha_de_comunicacao(exc):
                raise
            continue
        if not quadro_de_motor:
            return numero
        limite = time.monotonic() + prazo_s
        while True:
            try:
                houve_resposta = ler()
            except (RuntimeError, OSError) as exc:
                if not falha_de_comunicacao(exc):
                    raise
                break
            if houve_resposta:
                return numero
            if time.monotonic() >= limite:
                break
            esperar()
    return 0


def quadro_pode_repetir(quadro):
    """O ultimo comando MIT pode ser repetido. O quadro FD desliga o torque."""
    return bool(quadro) and b"t" in quadro and COMANDO_DESLIGA not in quadro


class LinkComReenvio(marcha.Link):
    """Na falha de quadro, reenvia o ultimo comando ate 3 vezes antes de desistir."""

    def __init__(self, port):
        super(LinkComReenvio, self).__init__(port)
        self.ultimo_quadro = b""
        self.recuperando = False
        self.recuperou_em = 0.0

    def write(self, data):
        if self.recuperando:
            marcha.Link.write(self, data)
            return
        if data:
            self.ultimo_quadro = data
        try:
            marcha.Link.write(self, data)
        except (RuntimeError, OSError) as exc:
            self._recuperar(exc, data)

    def poll(self):
        if self.recuperando:
            marcha.Link.poll(self)
            return
        try:
            self._poll_uma_vez()
        except (RuntimeError, OSError) as exc:
            self._recuperar(exc, self.ultimo_quadro or b"")

    def _poll_uma_vez(self):
        """Um byte de rejeicao ao lado de um quadro valido nao apaga esse quadro."""
        try:
            marcha.Link.poll(self)
        except (RuntimeError, OSError) as exc:
            if "CANable rejeitou" not in str(exc) or b"\x07" not in self.buffer:
                raise
            antes = dict(self.rx)
            self.buffer[:] = self.buffer.replace(b"\x07", b"")
            try:
                marcha.Link.poll(self)
            except (RuntimeError, OSError) as seguinte:
                if "CANable rejeitou" not in str(seguinte):
                    raise
                self.buffer[:] = self.buffer.replace(b"\x07", b"")
                raise exc
            if not any(self.rx[i] > antes.get(i, 0) for i in self.rx):
                raise exc

    def _recuperar(self, exc, quadro):
        if not falha_de_comunicacao(exc):
            raise
        if not quadro:
            raise RuntimeError(
                "Comunicacao nao voltou depois de %d reenvios: %s"
                % (TENTATIVAS_EXTRAS_CAN, exc))

        def escrever(dados):
            marcha.Link.write(self, dados)

        def ler():
            antes = dict(self.rx)
            marcha.Link.poll(self)
            return any(self.rx[i] > antes.get(i, 0) for i in self.rx)

        def limpar():
            self.buffer.clear()
            try:
                self.serial.reset_input_buffer()
            except Exception:
                pass

        self.recuperando = True
        try:
            numero = reenviar_ate_voltar(
                escrever, ler, limpar, quadro, b"t" in quadro)
        finally:
            self.recuperando = False
        if not numero:
            raise RuntimeError(
                "Comunicacao nao voltou depois de %d reenvios: %s"
                % (TENTATIVAS_EXTRAS_CAN, exc))
        self.recuperou_em = time.monotonic()
        print("CAN voltou no reenvio %d." % numero, flush=True)


def confirmar_leitura(contagens, chave, ativo):
    """True só depois de leituras seguidas. Uma leitura boa zera a contagem."""
    if not ativo:
        contagens.pop(chave, None)
        return False
    contagens[chave] = contagens.get(chave, 0) + 1
    return contagens[chave] >= LEITURAS_PARA_CONFIRMAR


def deve_interromper_por_atraso(atraso, duracao, recuperou):
    if recuperou:
        return False
    return atraso > ATRASO_LOOP_S or duracao > ATRASO_LOOP_S


def can_acabou_de_voltar(link, agora):
    return link.recuperou_em > 0.0 and agora - link.recuperou_em < JANELA_APOS_CAN_S


def velocidade_acima(link, can_id, estado):
    """True se a velocidade bruta ou o par recente do encoder passou de 3 rad/s.

    O monitor guarda o maior par da janela por ate 160 ms. Um unico quadro
    com posicao ruim deixa esse pico alto mesmo depois que o eixo acalmou.
    Por isso o pico so conta quando o par mais novo tambem passou do corte.
    """
    bruto = abs(estado["velocity_rad_s"])
    if bruto > marcha.RAW_SPEED_HARD_TRIP_RAD_S:
        return True
    estimado = estado.get("encoder_peak_speed_rad_s")
    if estimado is None or estimado <= caminhada.VELOCIDADE_CORTE_RAD_S:
        return False
    pontos = list(link.monitors[can_id].points)
    if len(pontos) < 2 or pontos[-1][0] <= pontos[-2][0]:
        return True
    ultimo = abs((pontos[-1][1] - pontos[-2][1]) / (pontos[-1][0] - pontos[-2][0]))
    return ultimo > caminhada.VELOCIDADE_CORTE_RAD_S


def esperar_quadro(link, can_id, prazo_s):
    """Le a serial ate chegar um quadro novo deste motor, ou devolve None."""
    limite = time.monotonic() + prazo_s
    while True:
        link.poll()
        estado = link.states.get(can_id)
        agora = time.monotonic()
        if estado is not None and agora - estado["received_at"] <= FEEDBACK_MAX_S:
            return agora
        if agora >= limite:
            return None
        time.sleep(0.005)


def reabrir_canal(link):
    """Fecha e abre o SLCAN sem trocar o ultimo comando MIT."""
    for comando in (b"C\r", b"O\r"):
        marcha.Link.write(link, comando)
        prazo = time.monotonic() + 0.03
        while time.monotonic() < prazo:
            try:
                link.poll()
            except (RuntimeError, OSError):
                link.buffer[:] = bytearray(link.buffer.replace(b"\x07", b""))
            time.sleep(0.002)


def recuperar_feedback(link, can_id):
    """Reenvia o ultimo MIT. Na segunda tentativa reabre o canal. Nao liga FD."""
    quadro = getattr(link, "ultimo_quadro", b"")
    if not quadro_pode_repetir(quadro):
        return None
    print(
        "ID %d: feedback atrasado. Repito o ultimo quadro antes de encerrar."
        % can_id,
        flush=True)
    for tentativa in range(1, TENTATIVAS_EXTRAS_CAN + 1):
        if tentativa == 2:
            print("ID %d: reabro o canal CAN." % can_id, flush=True)
            reabrir_canal(link)
        link.write(quadro)
        agora = esperar_quadro(link, can_id, PRAZO_RESPOSTA_CAN_S)
        if agora is None:
            continue
        return caminhada.ler_estado(
            link, can_id, agora, FEEDBACK_MAX_S, recusar_falha=False,
            recusar_velocidade=False, recusar_temperatura=False)
    return None


def ler_motor(link, can_id, agora, falhas, alertas):
    try:
        estado = caminhada.ler_estado(
            link, can_id, agora, FEEDBACK_MAX_S, recusar_falha=False,
            recusar_velocidade=False, recusar_temperatura=False)
    except RuntimeError as exc:
        if "feedback atrasado" not in str(exc):
            raise
        estado = recuperar_feedback(link, can_id)
        if estado is None:
            raise
    evento = acompanhar_falha(falhas, can_id, estado["fault"], agora)
    if evento == "temporaria":
        print(
            "ID %d: falha temporaria do driver (codigo %s). O movimento continua."
            % (can_id, estado["fault"]),
            flush=True)
    elif evento == "sumiu":
        print("ID %d: falha do driver sumiu." % can_id, flush=True)
    elif evento == "continua":
        raise RuntimeError(
            "ID %d: falha continua do driver (codigo %s) por mais de %.0f ms"
            % (can_id, estado["fault"], FEEDBACK_FALHA_CONTINUA_S * 1000))
    if confirmar_leitura(
            alertas, ("temperatura", can_id),
            estado["temperature_c"] >= caminhada.TEMPERATURA_CORTE_C):
        raise RuntimeError(
            "ID %d: temperatura %s °C em %d leituras seguidas"
            % (can_id, estado["temperature_c"], LEITURAS_PARA_CONFIRMAR))
    if confirmar_leitura(
            alertas, ("velocidade", can_id), velocidade_acima(link, can_id, estado)):
        raise RuntimeError(
            "ID %d: velocidade acima de %.0f rad/s em %d leituras seguidas"
            % (can_id, caminhada.VELOCIDADE_CORTE_RAD_S, LEITURAS_PARA_CONFIRMAR))
    return estado


def referencia_da_perna(joint, t, perna, profile):
    """Graus das duas pernas. perna e a que vai a frente.

    No gerador afinado os quadris recebem o mesmo numero. Como os motores
    sao espelhados, esse numero leva a esquerda para a frente e a direita
    para tras. O botao da direita inverte o sinal e o joelho da frente, para
    a direita ir a frente e a esquerda ir para tras.
    So o joelho da perna acionada flexiona. O da outra fica reto, em 0.
    """
    joelho_acionado = "knee_left" if perna == "esquerda" else "knee_right"
    if joint.name.startswith("knee_") and joint.name != joelho_acionado:
        return 0.0
    if perna == "esquerda":
        fonte = joint
        sinal = 1.0
    elif perna == "direita" and joint.name.startswith("hip_"):
        fonte = joint
        sinal = -1.0
    elif perna == "direita" and joint.name == "knee_right":
        fonte = JUNTAS["knee_left"]
        sinal = -1.0
    else:
        return 0.0
    return sinal * marcha.figure_steps_reference_deg(
        fonte, t, PASSOS_NO_GERADOR, "ambas", profile)


def problemas_de_batente(origens, profile, limites):
    problemas = []
    passo = caminhada.PERIODO_S
    amostras = int(round(DURACAO_PASSO_S / passo))
    for perna in ("esquerda", "direita"):
        for joint in marcha.JOINTS:
            baixo, alto = limites[joint.can_id]
            for indice in range(amostras + 1):
                graus = referencia_da_perna(
                    joint, indice * passo, perna, profile)
                alvo = origens[joint.can_id] + math.radians(graus)
                if baixo <= alvo <= alto:
                    continue
                problemas.append(
                    "ID %d no passo %s: alvo %+.2f° fora de [%+.2f°, %+.2f°]"
                    % (joint.can_id, perna, math.degrees(alvo),
                       math.degrees(baixo), math.degrees(alto)))
                break
    return problemas


def ampliar_limites_pela_origem(limites, origens):
    """A pose inicial vale. O usuario confere o angulo a olho na partida."""
    abertos = {}
    for can_id, (baixo, alto) in limites.items():
        origem = origens[can_id]
        abertos[can_id] = (min(baixo, origem), max(alto, origem))
    return abertos


class MaquinaPasso(object):
    """Qual perna pode andar, e so enquanto o botao dela permanece em 1.

    Os niveis que entram aqui ja passaram pelo filtro de 100 ms / 30 ms.
    Um botao preso em 1 desde a partida nao arma: precisa ser visto em 0.
    Os dois em 1 nao escolhem perna e nao continuam o passo: a referencia
    volta para em pe. No fim de um passo esse botao desliga ate o outro
    fechar o ciclo, e o proximo aperto tem que ser novo.
    Depois de 70% da trajetoria o passo segue ate o fim sem o botao.
    Os dois juntos interrompem isso e seguram em pe.
    """

    def __init__(self, duracao):
        self.duracao = duracao
        self.t = 0.0
        self.ativa = None
        self.feitas = set()
        self.armada = {"esquerda": False, "direita": False}
        self.aviso_dois = False
        self.segue_sozinho = False
        self.ciclos = 0

    def habilitada(self, perna):
        return perna not in self.feitas

    def abortar(self, niveis):
        """Desiste do passo atual sem contar. Exige soltar e apertar de novo."""
        self.ativa = None
        self.t = 0.0
        self.segue_sozinho = False
        self._desarmar_apertados(niveis)

    def atualizar(self, niveis, dt):
        eventos = []
        for perna in ("esquerda", "direita"):
            if niveis[perna] == 0:
                self.armada[perna] = True
                self.aviso_dois = False

        if niveis["esquerda"] == 1 and niveis["direita"] == 1:
            cancelou = self.ativa is not None
            if cancelou:
                self.ativa = None
                self.t = 0.0
                self.segue_sozinho = False
                self._desarmar_apertados(niveis)
            if cancelou or self.armada["esquerda"] or self.armada["direita"]:
                if not self.aviso_dois:
                    self.aviso_dois = True
                    eventos.append("dois_botoes")
            return self._em_pe(eventos)

        if self.ativa is not None:
            perna = self.ativa
            if ((niveis[perna] == 1 or self.segue_sozinho)
                    and dt > 0.0 and self.t < self.duracao):
                self.t = min(self.duracao, self.t + dt)
                avancando = True
            else:
                avancando = False
            limite = self.duracao * FRACAO_PARA_CONCLUIR
            if self.t >= limite and self.t < self.duracao and not self.segue_sozinho:
                self.segue_sozinho = True
                eventos.append("segue_sozinho_" + perna)
            if self.t >= self.duracao:
                self._concluir(niveis, eventos)
                return {
                    "perna": perna,
                    "t": self.duracao,
                    "avancando": False,
                    "eventos": eventos,
                }
            return {
                "perna": perna,
                "t": self.t,
                "avancando": avancando,
                "eventos": eventos,
            }

        candidatos = []
        for perna in ("esquerda", "direita"):
            if niveis[perna] != 1 or not self.armada[perna]:
                continue
            if not self.habilitada(perna):
                self.armada[perna] = False
                eventos.append("botao_desabilitado_" + perna)
                continue
            candidatos.append(perna)
        if len(candidatos) >= 2:
            if not self.aviso_dois:
                self.aviso_dois = True
                eventos.append("dois_botoes")
            return self._em_pe(eventos)
        if len(candidatos) == 1:
            self.ativa = candidatos[0]
            self.t = 0.0
            self.segue_sozinho = False
            eventos.append("inicio_" + self.ativa)
            return {
                "perna": self.ativa,
                "t": 0.0,
                "avancando": False,
                "eventos": eventos,
            }
        return self._em_pe(eventos)

    def _concluir(self, niveis, eventos):
        feita = self.ativa
        self.feitas.add(feita)
        self.ativa = None
        self.t = 0.0
        self.segue_sozinho = False
        self._desarmar_apertados(niveis)
        if len(self.feitas) >= 2:
            self.feitas.clear()
            self.ciclos += 1
            eventos.append("ciclo_fechado")
        else:
            eventos.append("passo_concluido_" + feita)

    def _desarmar_apertados(self, niveis):
        for perna in ("esquerda", "direita"):
            if niveis[perna] == 1:
                self.armada[perna] = False

    def _em_pe(self, eventos):
        return {"perna": None, "t": 0.0, "avancando": False, "eventos": eventos}


def pico_velocidade_rad_s(profile):
    passo = caminhada.PERIODO_S
    amostras = int(round(DURACAO_PASSO_S / passo))
    pico = 0.0
    for perna in ("esquerda", "direita"):
        anterior = {
            joint.can_id: referencia_da_perna(joint, 0.0, perna, profile)
            for joint in marcha.JOINTS}
        for indice in range(1, amostras + 1):
            t = min(DURACAO_PASSO_S, indice * passo)
            for joint in marcha.JOINTS:
                atual = referencia_da_perna(joint, t, perna, profile)
                delta = abs(math.radians(atual - anterior[joint.can_id]) / passo)
                if delta > pico:
                    pico = delta
                anterior[joint.can_id] = atual
    return pico


def checar_logica():
    """Curva, velocidade e sequencia dos botoes. Nao abre CAN nem GPIO."""
    profile = caminhada.perfil_da_caminhada()
    passo = caminhada.PERIODO_S
    amostras = int(round(DURACAO_PASSO_S / passo))
    for perna in ("esquerda", "direita"):
        for joint in marcha.JOINTS:
            exigir(
                referencia_da_perna(joint, 0.0, perna, profile) == 0.0,
                "O passo %s nao comeca em pe na junta %s" % (perna, joint.name))
            exigir(
                referencia_da_perna(joint, DURACAO_PASSO_S, perna, profile) == 0.0,
                "O passo %s nao termina em pe na junta %s" % (perna, joint.name))
        faixa = {joint.name: [0.0, 0.0] for joint in marcha.JOINTS}
        for indice in range(amostras + 1):
            t = min(DURACAO_PASSO_S, indice * passo)
            for joint in marcha.JOINTS:
                graus = referencia_da_perna(joint, t, perna, profile)
                faixa[joint.name][0] = min(faixa[joint.name][0], graus)
                faixa[joint.name][1] = max(faixa[joint.name][1], graus)
                exigir(
                    -45.0 - 1e-6 <= graus <= 45.0 + 1e-6,
                    "Angulo fora da marcha afinada: %s %+.2f°" % (joint.name, graus))
        exigir(
            math.isclose(faixa["hip_left"][0], faixa["hip_right"][0])
            and math.isclose(faixa["hip_left"][1], faixa["hip_right"][1]),
            "Os quadris do passo %s nao andam juntos" % perna)
        if perna == "esquerda":
            exigir(faixa["hip_left"][1] > 15.0, "A esquerda nao vai a frente")
            exigir(faixa["hip_right"][1] > 15.0, "A direita nao vai para tras com a esquerda")
            exigir(faixa["knee_left"][1] > 40.0, "O joelho da frente nao flexiona")
            exigir(
                math.isclose(faixa["knee_right"][0], 0.0)
                and math.isclose(faixa["knee_right"][1], 0.0),
                "O joelho de tras nao ficou reto")
        else:
            exigir(faixa["hip_right"][0] < -15.0, "A direita nao vai a frente")
            exigir(faixa["hip_left"][0] < -15.0, "A esquerda nao vai para tras com a direita")
            exigir(faixa["knee_right"][0] < -40.0, "O joelho da frente nao flexiona")
            exigir(
                math.isclose(faixa["knee_left"][0], 0.0)
                and math.isclose(faixa["knee_left"][1], 0.0),
                "O joelho de tras nao ficou reto")
    pico = pico_velocidade_rad_s(profile)
    if pico > caminhada.VELOCIDADE_REFERENCIA_MAX_RAD_S + 1e-6:
        raise RuntimeError(
            "Velocidade de referencia %.3f rad/s passa de %g rad/s"
            % (pico, caminhada.VELOCIDADE_REFERENCIA_MAX_RAD_S))

    exigir(math.isclose(FEEDBACK_MAX_S, 0.5), "O limite de feedback mudou")
    exigir(math.isclose(FEEDBACK_FALHA_CONTINUA_S, 0.5), "O tempo de falha continua mudou")
    exigir(math.isclose(FOLGA_BATENTE_GRAUS, 2.0), "A folga do batente mudou")
    exigir(math.isclose(ATRASO_LOOP_S, 0.20), "O atraso de loop mudou")
    exigir(TENTATIVAS_EXTRAS_CAN == 3, "Os reenvios da CAN mudaram")
    exigir(LEITURAS_PARA_CONFIRMAR == 3, "As leituras para confirmar mudaram")
    exigir(math.isclose(JANELA_APOS_CAN_S, 1.0), "A janela depois da CAN mudou")
    exigir(math.isclose(FRACAO_PARA_CONCLUIR, 0.70), "A fracao para concluir o passo mudou")
    exigir(GPIO_ATIVO is False, "O GPIO desta versao Windows ficou ativo")
    exigir(LED_GPIO is None and LED_PINO is None, "O LED de GPIO ficou ativo no Windows")
    exigir(math.isclose(PRAZO_RESPOSTA_CAN_S, 0.10), "O prazo de resposta da CAN mudou")
    exigir(not quadro_pode_repetir(b""), "Quadro vazio pode ser repetido")
    exigir(not quadro_pode_repetir(b"C\r"), "Comando de abertura pode ser repetido como motor")
    exigir(not quadro_pode_repetir(b"t0018" + COMANDO_DESLIGA + b"\r"),
           "O quadro que desliga o torque pode ser repetido")
    exigir(quadro_pode_repetir(b"t00181122334455667788\r"),
           "O quadro MIT foi recusado na repeticao")
    contagens = {}
    chave = ("velocidade", 1)
    exigir(not confirmar_leitura(contagens, chave, True), "Uma leitura ja encerrou")
    exigir(not confirmar_leitura(contagens, chave, True), "Duas leituras ja encerraram")
    exigir(confirmar_leitura(contagens, chave, True), "Tres leituras nao confirmaram")
    exigir(not confirmar_leitura(contagens, chave, False), "Leitura boa nao zerou o alerta")
    exigir(chave not in contagens, "O alerta zerado ficou contando")
    exigir(not deve_interromper_por_atraso(0.19, 0.19, False), "Atraso de 190 ms encerrou")
    exigir(deve_interromper_por_atraso(0.21, 0.0, False), "Chegada atrasada 210 ms passou")
    exigir(deve_interromper_por_atraso(0.0, 0.21, False), "Trabalho de 210 ms passou")
    exigir(
        not deve_interromper_por_atraso(1.0, 1.0, True),
        "Atraso logo apos a CAN voltar encerrou")
    falhas = {}
    exigir(acompanhar_falha(falhas, 3, 0, 0.0) is None, "Sem falha nao pode acusar")
    exigir(acompanhar_falha(falhas, 3, 4, 0.0) == "temporaria", "A primeira falha tem que ser temporaria")
    exigir(acompanhar_falha(falhas, 3, 4, 0.49) is None, "Falha de 490 ms ainda e temporaria")
    exigir(acompanhar_falha(falhas, 3, 4, 0.50) == "continua", "Falha de 500 ms tem que encerrar")
    falhas = {}
    exigir(acompanhar_falha(falhas, 3, 4, 1.0) == "temporaria", "A falha nova nao armou")
    exigir(acompanhar_falha(falhas, 3, 0, 1.1) == "sumiu", "A falha que passou nao foi esquecida")
    exigir(acompanhar_falha(falhas, 3, 4, 1.2) == "temporaria", "O temporizador da falha nao zerou")

    class Canal(object):
        def __init__(self, falhas, responde):
            self.falhas = falhas
            self.responde = responde
            self.envios = 0

        def escrever(self, _quadro):
            self.envios += 1
            if self.falhas:
                self.falhas -= 1
                raise RuntimeError("Escrita serial incompleta")

        def ler(self):
            return self.responde

        def limpar(self):
            return None

    imediato = Canal(0, True)
    exigir(
        reenviar_ate_voltar(
            imediato.escrever, imediato.ler, imediato.limpar, b"t", True, lambda: None, 0) == 1,
        "O primeiro reenvio com resposta nao voltou")
    teimoso = Canal(2, True)
    exigir(
        reenviar_ate_voltar(
            teimoso.escrever, teimoso.ler, teimoso.limpar, b"t", True, lambda: None, 0) == 3,
        "O terceiro reenvio nao foi aceito")
    mudo = Canal(0, False)
    exigir(
        reenviar_ate_voltar(
            mudo.escrever, mudo.ler, mudo.limpar, b"t", True, lambda: None, 0) == 0,
        "Quadro de motor sem resposta nao pode contar como volta")
    texto = Canal(0, False)
    exigir(
        reenviar_ate_voltar(
            texto.escrever, texto.ler, texto.limpar, b"C\r", False, lambda: None, 0) == 1,
        "Comando de abertura da CAN exigiu resposta de motor")

    class CanalUsb(object):
        def __init__(self):
            self.envios = 0

        def escrever(self, _quadro):
            self.envios += 1
            if self.envios < 3:
                raise OSError("write timeout")

        def ler(self):
            return True

        def limpar(self):
            return None

    usb = CanalUsb()
    exigir(
        reenviar_ate_voltar(
            usb.escrever, usb.ler, usb.limpar, b"t", True, lambda: None, 0) == 3,
        "Timeout USB nao foi reenviado")

    class CanalTarde(object):
        def __init__(self):
            self.envios = 0
            self.leituras = 0

        def escrever(self, _quadro):
            self.envios += 1

        def ler(self):
            self.leituras += 1
            return self.leituras >= 3

        def limpar(self):
            return None

    tarde = CanalTarde()
    exigir(
        reenviar_ate_voltar(
            tarde.escrever, tarde.ler, tarde.limpar, b"t", True, lambda: None, 1.0) == 1,
        "Resposta atrasada nao contou no mesmo reenvio")
    exigir(tarde.envios == 1, "A resposta atrasada gerou outro envio")

    class SerialFalso(object):
        def __init__(self, blocos):
            self.blocos = list(blocos)

        @property
        def in_waiting(self):
            return len(self.blocos[0]) if self.blocos else 0

        def read(self, _n):
            if not self.blocos:
                return b""
            return self.blocos.pop(0)

        def write(self, dados):
            return len(dados)

        def reset_input_buffer(self):
            self.blocos = []

    def enlace_falso(blocos, quadro):
        enlace = LinkComReenvio.__new__(LinkComReenvio)
        enlace.buffer = bytearray()
        enlace.states = {}
        enlace.tx = 0
        enlace.rx = {joint.can_id: 0 for joint in marcha.JOINTS}
        enlace.monitors = {joint.can_id: marcha.EncoderMonitor() for joint in marcha.JOINTS}
        enlace.cancel_file = None
        enlace.ultimo_quadro = quadro
        enlace.recuperando = False
        enlace.recuperou_em = 0.0
        enlace.serial = SerialFalso(blocos)
        return enlace

    resposta = b"t0018017FFF0000004100\r"
    exigir(len(resposta) == 22, "O quadro de teste do motor 1 mudou de tamanho")
    com_sino = enlace_falso([b"\x07" + resposta], b"t00181122334455667788\r")
    com_sino.poll()
    exigir(com_sino.rx[1] == 1, "O sino apagou o quadro valido do motor")
    exigir(com_sino.recuperou_em == 0.0, "O sino ao lado de um quadro valido entrou em reenvio")
    exigir(b"\x07" not in com_sino.buffer, "O sino ficou no buffer")

    so_sino = enlace_falso([b"\x07"], b"C\r")
    so_sino.poll()
    exigir(so_sino.recuperou_em > 0.0, "Rejeicao na abertura da CAN encerrou sem reenvio")
    exigir(
        falha_de_comunicacao(OSError("write timeout")),
        "OSError nao entrou como falha de comunicacao")
    exigir(
        not falha_de_comunicacao(RuntimeError("ID 1: temperatura 70 °C")),
        "Temperatura foi tratada como falha de CAN")

    class MonitorFalso(object):
        def __init__(self, pontos):
            self.points = pontos

    class LinkFalso(object):
        def __init__(self, pontos):
            self.monitors = {1: MonitorFalso(pontos)}

    calmo = LinkFalso([(0.00, 0.0), (0.02, 0.0), (0.04, 0.5), (0.06, 0.5)])
    pico_velho = {
        "velocity_rad_s": 0.2,
        "encoder_peak_speed_rad_s": 25.0,
    }
    exigir(
        not velocidade_acima(calmo, 1, pico_velho),
        "Pico velho do encoder encerrou com o eixo calmo")
    rapido = LinkFalso([(0.00, 0.0), (0.02, 0.0), (0.04, 0.2)])
    exigir(
        velocidade_acima(rapido, 1, pico_velho),
        "Par recente acima de 3 rad/s nao contou")
    bruto_alto = {"velocity_rad_s": -3.1, "encoder_peak_speed_rad_s": None}
    exigir(
        velocidade_acima(LinkFalso([]), 1, bruto_alto),
        "Velocidade bruta acima de 3 rad/s nao contou")
    dentro = {"velocity_rad_s": 1.0, "encoder_peak_speed_rad_s": 1.2}
    exigir(
        not velocidade_acima(LinkFalso([]), 1, dentro),
        "Velocidade dentro do corte foi recusada")
    exigir(math.isclose(gpio.CONFIRMA_UM_S, 0.10), "A confirmacao de nivel 1 mudou")
    exigir(math.isclose(gpio.CONFIRMA_ZERO_S, 0.03), "A confirmacao de nivel 0 mudou")
    filtro = gpio.Filtro()
    exigir(filtro.atualizar(1, 0.0) == 0, "Nivel 1 nao pode entrar no mesmo instante")
    exigir(filtro.atualizar(1, 0.09) == 0, "90 ms de nivel 1 ainda e ruído")
    exigir(filtro.atualizar(1, 0.10) == 1, "100 ms de nivel 1 tem que valer")
    exigir(filtro.atualizar(0, 0.12) == 1, "20 ms em 0 nao pode soltar o botao")
    exigir(filtro.atualizar(0, 0.15) == 0, "30 ms em 0 tem que soltar o botao")

    maquina = MaquinaPasso(0.05)
    preso = maquina.atualizar({"esquerda": 1, "direita": 1}, 0.02)
    exigir(preso["perna"] is None and not preso["eventos"], "Botao preso na partida moveu")

    maquina.atualizar({"esquerda": 0, "direita": 0}, 0.02)
    inicio = maquina.atualizar({"esquerda": 1, "direita": 0}, 0.02)
    exigir(inicio["eventos"] == ["inicio_esquerda"], "A esquerda nao iniciou")
    exigir(inicio["t"] == 0.0 and inicio["avancando"] is False, "O inicio ja andou")
    andou = maquina.atualizar({"esquerda": 1, "direita": 0}, 0.02)
    exigir(
        andou["perna"] == "esquerda" and andou["avancando"] is True
        and math.isclose(andou["t"], 0.02),
        "O botao da esquerda nao avancou o passo")
    parado = maquina.atualizar({"esquerda": 0, "direita": 1}, 0.02)
    exigir(
        parado["avancando"] is False and math.isclose(parado["t"], 0.02)
        and maquina.ativa == "esquerda",
        "Soltar o botao tem que congelar o passo")
    seguiu = maquina.atualizar({"esquerda": 1, "direita": 0}, 0.02)
    exigir(math.isclose(seguiu["t"], 0.04) and seguiu["avancando"] is True, "O aperto nao retomou")
    sem_dt = maquina.atualizar({"esquerda": 1, "direita": 0}, 0.0)
    exigir(math.isclose(sem_dt["t"], 0.04) and sem_dt["avancando"] is False, "Pausa de feedback andou")
    fechou = maquina.atualizar({"esquerda": 1, "direita": 0}, 1.0)
    exigir(
        fechou["eventos"] == ["passo_concluido_esquerda"]
        and math.isclose(fechou["t"], 0.05) and fechou["perna"] == "esquerda",
        "O fim do passo esquerdo nao fechou a curva")
    exigir(maquina.ativa is None and maquina.feitas == {"esquerda"}, "A esquerda continuou habilitada")
    ainda = maquina.atualizar({"esquerda": 1, "direita": 1}, 0.02)
    exigir(ainda["perna"] is None, "Botao ainda apertado no fim disparou a outra perna")
    maquina.atualizar({"esquerda": 0, "direita": 0}, 0.02)
    recusa = maquina.atualizar({"esquerda": 1, "direita": 0}, 0.02)
    exigir(
        recusa["eventos"] == ["botao_desabilitado_esquerda"] and recusa["perna"] is None,
        "A esquerda andou de novo antes da direita")
    maquina.atualizar({"esquerda": 0, "direita": 0}, 0.02)
    os_dois = maquina.atualizar({"esquerda": 1, "direita": 1}, 0.02)
    exigir(
        os_dois["perna"] is None and os_dois["eventos"] == ["dois_botoes"],
        "Os dois botoes andaram com a esquerda desabilitada")
    maquina.atualizar({"esquerda": 0, "direita": 0}, 0.02)
    direita = maquina.atualizar({"esquerda": 0, "direita": 1}, 0.02)
    exigir(direita["eventos"] == ["inicio_direita"], "A direita nao fechou o ciclo")
    ciclo = maquina.atualizar({"esquerda": 0, "direita": 1}, 0.05)
    exigir(ciclo["eventos"] == ["ciclo_fechado"] and maquina.ciclos == 1, "O ciclo nao fechou")
    exigir(not maquina.feitas and maquina.ativa is None, "Os botoes nao voltaram juntos")
    segurou = maquina.atualizar({"esquerda": 1, "direita": 1}, 0.02)
    exigir(
        segurou["perna"] is None and segurou["eventos"] == ["dois_botoes"],
        "Botao ainda apertado depois do ciclo andou")
    maquina.atualizar({"esquerda": 0, "direita": 0}, 0.02)
    depois = maquina.atualizar({"esquerda": 1, "direita": 1}, 0.02)
    exigir(
        depois["perna"] is None and depois["eventos"] == ["dois_botoes"],
        "Os dois botoes andaram")
    repetiu = maquina.atualizar({"esquerda": 1, "direita": 1}, 0.02)
    exigir(repetiu["perna"] is None and not repetiu["eventos"], "O aviso dos dois botoes repetiu")
    so_direita = maquina.atualizar({"esquerda": 0, "direita": 1}, 0.02)
    exigir(so_direita["eventos"] == ["inicio_direita"], "Soltar um dos dois nao escolheu a perna")

    no_meio = MaquinaPasso(1.0)
    no_meio.atualizar({"esquerda": 0, "direita": 0}, 0.02)
    no_meio.atualizar({"esquerda": 1, "direita": 0}, 0.02)
    no_meio.atualizar({"esquerda": 1, "direita": 0}, 0.40)
    segurou_dois = no_meio.atualizar({"esquerda": 1, "direita": 1}, 0.02)
    exigir(
        segurou_dois["perna"] is None and segurou_dois["avancando"] is False
        and segurou_dois["eventos"] == ["dois_botoes"]
        and no_meio.ativa is None and not no_meio.feitas,
        "Os dois botoes durante o passo continuaram a caminhada")
    ainda_dois = no_meio.atualizar({"esquerda": 1, "direita": 1}, 0.50)
    exigir(
        ainda_dois["perna"] is None and not ainda_dois["eventos"]
        and ainda_dois["avancando"] is False,
        "Os dois botoes seguiram andando na espera do desligamento")
    exigiu_novo = no_meio.atualizar({"esquerda": 1, "direita": 0}, 0.02)
    exigir(
        exigiu_novo["perna"] is None and not no_meio.feitas,
        "Soltar um botao depois dos dois retomou o passo sem aperto novo")

    for perna, outra in (("esquerda", "direita"), ("direita", "esquerda")):
        livre = {"esquerda": 0, "direita": 0}
        apertado = {"esquerda": 0, "direita": 0}
        apertado[perna] = 1
        meio = MaquinaPasso(1.0)
        meio.atualizar(livre, 0.02)
        meio.atualizar(apertado, 0.02)
        antes = meio.atualizar(apertado, 0.69)
        exigir(
            math.isclose(antes["t"], 0.69) and "segue_sozinho_" + perna not in antes["eventos"],
            "O aviso de 70%% saiu cedo na perna %s" % perna)
        solto = meio.atualizar(livre, 0.02)
        exigir(
            solto["avancando"] is False and math.isclose(solto["t"], 0.69)
            and meio.ativa == perna,
            "Soltar antes de 70%% nao congelou a perna %s" % perna)
        cruzou = meio.atualizar(apertado, 0.02)
        exigir(
            cruzou["avancando"] is True and math.isclose(cruzou["t"], 0.71)
            and cruzou["eventos"] == ["segue_sozinho_" + perna],
            "Passar de 70%% nao liberou a perna %s" % perna)
        roubou = {"esquerda": 0, "direita": 0}
        roubou[outra] = 1
        seguiu = meio.atualizar(roubou, 0.02)
        exigir(
            seguiu["perna"] == perna and seguiu["avancando"] is True
            and math.isclose(seguiu["t"], 0.73) and not seguiu["eventos"],
            "Depois de 70%% soltar nao completou a perna %s" % perna)
        pausa = meio.atualizar(livre, 0.0)
        exigir(
            pausa["avancando"] is False and math.isclose(pausa["t"], 0.73),
            "A pausa de feedback andou depois de 70%% na perna %s" % perna)
        fim = meio.atualizar(livre, 1.0)
        exigir(
            fim["eventos"] == ["passo_concluido_" + perna] and meio.ativa is None,
            "O passo %s nao terminou sozinho" % perna)
        exigir(not meio.segue_sozinho, "O passo concluido ficou marcado para seguir sozinho")

    corte = MaquinaPasso(1.0)
    corte.atualizar({"esquerda": 0, "direita": 0}, 0.02)
    corte.atualizar({"esquerda": 1, "direita": 0}, 0.02)
    corte.atualizar({"esquerda": 1, "direita": 0}, 0.70)
    exigir(corte.segue_sozinho, "O aborto de teste nao chegou a 70%")
    corte.abortar({"esquerda": 1, "direita": 0})
    exigir(
        corte.ativa is None and not corte.segue_sozinho and not corte.feitas,
        "Abortar depois de 70% continuou ou contou o passo")
    exigir(corte.atualizar({"esquerda": 1, "direita": 0}, 0.02)["perna"] is None,
           "O aborto depois de 70% reiniciou sem soltar")

    outra = MaquinaPasso(0.05)
    outra.atualizar({"esquerda": 0, "direita": 0}, 0.02)
    outra.atualizar({"esquerda": 0, "direita": 1}, 0.02)
    outra.abortar({"esquerda": 0, "direita": 1})
    exigir(outra.ativa is None and not outra.feitas, "O passo abortado foi contado")
    ficou = outra.atualizar({"esquerda": 0, "direita": 1}, 0.02)
    exigir(ficou["perna"] is None, "O aborto reiniciou sem um aperto novo")
    outra.atualizar({"esquerda": 0, "direita": 0}, 0.02)
    de_novo = outra.atualizar({"esquerda": 0, "direita": 1}, 0.02)
    exigir(de_novo["eventos"] == ["inicio_direita"], "A direita nao retomou depois do aborto")

    medido = {2: (math.radians(0.47), math.radians(91.70))}
    dentro = ampliar_limites_pela_origem(medido, {2: math.radians(10.0)})
    exigir(
        math.isclose(dentro[2][0], medido[2][0])
        and math.isclose(dentro[2][1], medido[2][1]),
        "Origem dentro do batente mudou o limite")
    fora = ampliar_limites_pela_origem(medido, {2: math.radians(-40.0)})
    exigir(math.isclose(fora[2][0], math.radians(-40.0)), "Origem fora nao abriu o batente")
    exigir(math.isclose(fora[2][1], medido[2][1]), "O teto do batente mudou sem necessidade")

    pulso = gpio.Filtro()
    segura = MaquinaPasso(1.0)
    relogio = 0.0
    pulso.atualizar(0, relogio)
    segura.atualizar({"esquerda": 0, "direita": 0}, 0.02)
    for _ in range(5):
        relogio += 0.01
        nivel = pulso.atualizar(1, relogio)
        visto = segura.atualizar({"esquerda": nivel, "direita": 0}, 0.02)
        exigir(visto["perna"] is None, "Um pulso curto de 50 ms iniciou o passo")
    checar_abertura_da_porta()
    return pico


def checar_abertura_da_porta():
    """A espera da porta nao abre a CAN real."""
    import serial

    exigir(math.isclose(ESPERA_PORTA_S, 60.0), "A espera da porta CAN mudou")
    exigir(math.isclose(PAUSA_PORTA_S, 0.5), "A pausa da porta CAN mudou")
    exigir(porta_merece_espera(serial.SerialException(errno.EBUSY, "ocupada")),
           "Porta ocupada nao espera")
    exigir(porta_merece_espera(OSError(errno.ENOENT, "ausente")),
           "Porta ausente nao espera")
    exigir(not porta_merece_espera(OSError(errno.EIO, "quebra")),
           "Erro de entrada e saida ficou esperando")
    exigir(not porta_merece_espera(RuntimeError("sem resposta")),
           "Falha de motor ficou esperando a porta")
    class SemQuadro(object):
        ultimo_quadro = b""
    exigir(recuperar_feedback(SemQuadro(), 1) is None,
           "Sem comando MIT a espera de feedback reenviou")
    class QuadroDeDesliga(object):
        ultimo_quadro = b"t0018" + COMANDO_DESLIGA + b"\r"
    exigir(recuperar_feedback(QuadroDeDesliga(), 2) is None,
           "O quadro FD foi repetido para buscar feedback")

    class Relogio(object):
        def __init__(self):
            self.t = 0.0
            self.dormiu = []

        def agora(self):
            return self.t

        def dormir(self, segundos):
            self.dormiu.append(segundos)
            self.t += segundos

    relogio = Relogio()
    chamadas = []
    avisos = []
    piscadas = []

    def abre_na_terceira():
        chamadas.append(relogio.t)
        if len(chamadas) < 3:
            raise serial.SerialException(errno.EBUSY, "ocupada")

    abrir_quando_livre(
        abre_na_terceira, prazo_s=60.0, pausa_s=0.5,
        agora=relogio.agora, dormir=relogio.dormir,
        avisar=avisos.append, enquanto_espera=lambda: piscadas.append(None))
    exigir(chamadas == [0.0, 0.5, 1.0], "A porta ocupada nao foi tentada de novo")
    exigir(relogio.dormiu == [0.5, 0.5], "A espera da porta dormiu o tempo errado")
    exigir(len(avisos) == 2 and avisos[1] == "Porta CAN livre.",
           "O aviso da porta ocupada nao fechou")
    exigir(piscadas == [None, None], "O LED nao piscou enquanto a porta esperava")

    relogio = Relogio()

    def ja_livre():
        return None

    abrir_quando_livre(
        ja_livre, prazo_s=60.0, agora=relogio.agora, dormir=relogio.dormir,
        avisar=avisos.append)
    exigir(not relogio.dormiu, "A porta livre esperou")

    relogio = Relogio()

    def sempre_ocupada():
        raise OSError(errno.EBUSY, "ocupada")

    try:
        abrir_quando_livre(
            sempre_ocupada, prazo_s=1.0, pausa_s=0.5,
            agora=relogio.agora, dormir=relogio.dormir)
    except OSError as exc:
        exigir(exc.errno == errno.EBUSY, "O prazo da porta nao devolveu a ocupacao")
    else:
        raise RuntimeError("A porta ocupada alem do prazo abriu")
    exigir(relogio.t == 1.0, "A espera passou do prazo")

    def quebra():
        raise OSError(errno.EIO, "quebra")

    try:
        abrir_quando_livre(
            quebra, prazo_s=60.0, agora=relogio.agora, dormir=relogio.dormir)
    except OSError as exc:
        exigir(exc.errno == errno.EIO, "Outro erro da porta foi trocado")
    else:
        raise RuntimeError("Erro permanente da porta foi engolido")


def preparar_botoes():
    if GPIO_ATIVO:
        raise RuntimeError("O GPIO desta versao Windows nao pode ser ligado")
    return {nome: gpio.Filtro() for nome, _pino, _numero in ENTRADAS}


def ler_botoes(filtros, agora):
    """Devolve (estavel, bruto, leitura_ok). Falha forca 0 nas duas."""
    bruto = {}
    estavel = {}
    try:
        for nome, _pino, numero in ENTRADAS:
            bruto[nome] = gpio.ler_bruto(numero)
            estavel[nome] = filtros[nome].atualizar(bruto[nome], agora)
    except OSError as exc:
        for nome, _pino, _numero in ENTRADAS:
            filtros[nome].estavel = 0
            filtros[nome].bruto = 0
            filtros[nome].desde = agora
        return (
            {"esquerda": 0, "direita": 0},
            {"esquerda": 0, "direita": 0},
            False,
            str(exc),
        )
    return estavel, bruto, True, ""


def descrever(evento):
    textos = {
        "inicio_esquerda":
            "ESQUERDA a frente, DIREITA para tras. Segure o pino 31. Depois de 70% pode soltar.",
        "inicio_direita":
            "DIREITA a frente, ESQUERDA para tras. Segure o pino 33. Depois de 70% pode soltar.",
        "segue_sozinho_esquerda":
            "70% da esquerda. Pode soltar o pino 31. O passo termina sozinho.",
        "segue_sozinho_direita":
            "70% da direita. Pode soltar o pino 33. O passo termina sozinho.",
        "passo_concluido_esquerda":
            "ESQUERDA terminou. Botao do pino 31 desabilitado. "
            "Segurando em pe. Aperte o pino 33 para fechar o passo.",
        "passo_concluido_direita":
            "DIREITA terminou. Botao do pino 33 desabilitado. "
            "Segurando em pe. Aperte o pino 31 para fechar o passo.",
        "ciclo_fechado":
            "Passo completo fechado. Exo em pe. Os dois botoes voltam a valer. "
            "Solte e aperte de novo.",
        "botao_desabilitado_esquerda":
            "Pino 31 desabilitado. Falta a perna direita para fechar o passo.",
        "botao_desabilitado_direita":
            "Pino 33 desabilitado. Falta a perna esquerda para fechar o passo.",
        "dois_botoes":
            "Os dois botoes juntos nao movem. Exo em pe.",
    }
    return textos.get(evento, evento)


def texto_botoes(maquina):
    partes = []
    nomes = {"esquerda": "Esquerda", "direita": "Direita"}
    for nome, _pino, _numero in ENTRADAS:
        if maquina.ativa == nome:
            estado = "em passo"
        elif maquina.ativa is not None:
            estado = "ignorado ate o fim deste passo"
        elif maquina.habilitada(nome):
            estado = "livre"
        else:
            estado = "desabilitado"
        partes.append("%s %s" % (nomes[nome], estado))
    return " | ".join(partes)


def pedir_feedback(link, enviar, fase, mensagem):
    """Pede quadro aos quatro motores. Se faltar resposta, tenta a leva inteira de novo."""
    ids = {joint.can_id for joint in marcha.JOINTS}
    faltando, tentativas = marcha.request_fresh_feedback(
        link, marcha.JOINTS, ids, enviar, phase=fase)
    if not faltando:
        return tentativas
    print(
        "Sem resposta %s dos IDs %s. Tento outra vez." % (fase, sorted(faltando)),
        flush=True)
    faltando, extra = marcha.request_fresh_feedback(
        link, marcha.JOINTS, ids, enviar, phase=fase + "_outra")
    tentativas += extra
    if faltando:
        raise RuntimeError(mensagem % (sorted(faltando), tentativas))
    return tentativas


def segurar_neutro(link, ciclos):
    """MIT com Kp zero. Habilita o modo e fecha a janela do encoder sem puxar a pose."""
    for _ in range(ciclos):
        link.write(b"".join(
            marcha.frame(joint.can_id, caminhada.comando_neutro(joint))
            for joint in marcha.JOINTS))
        prazo = time.monotonic() + caminhada.PERIODO_S
        while time.monotonic() < prazo:
            link.poll()
            time.sleep(0.001)


def ler_repouso(link, falhas, alertas):
    """Origens em repouso. Esforco alto ganha mais um periodo. Janela vazia devolve o ID."""
    agora = time.monotonic()
    origens = {}
    for joint in marcha.JOINTS:
        estado = ler_motor(link, joint.can_id, agora, falhas, alertas)
        if abs(estado["effort_nm"]) >= caminhada.ESFORCO_REPOUSO_MAX_NM:
            segurar_neutro(link, 1)
            agora = time.monotonic()
            estado = ler_motor(link, joint.can_id, agora, falhas, alertas)
            if abs(estado["effort_nm"]) >= caminhada.ESFORCO_REPOUSO_MAX_NM:
                raise RuntimeError(
                    "ID %d: esforco em repouso %+.2f Nm"
                    % (joint.can_id, estado["effort_nm"]))
        if estado.get("encoder_peak_speed_rad_s") is None:
            return None, joint.can_id
        origens[joint.can_id] = estado["position_rad"]
    return origens, None


def interromper_se_desligar(pedido):
    if pedido is not None and pedido.desligar:
        raise KeyboardInterrupt("desligamento")


def _publicar(estado, **campos):
    if estado is not None:
        estado.definir(**campos)


def executar(porta=None, led=None, pedido=None, estado=None, sinais=True):
    pico_velocidade = checar_logica()
    filtros = preparar_botoes()
    porta = caminhada.resolver_porta(porta)
    profile = caminhada.perfil_da_caminhada()
    meta = profile[2]
    intensidade = 1.0
    caminhada.aplicar_intensidade(meta, intensidade)
    limites = caminhada.limites_rad()
    pasta = Path(__file__).resolve().parent / "ensaios" / (
        "passo_por_botao_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    pasta.mkdir(parents=True, exist_ok=True)
    relatorio = {
        "modo": "passo_por_botao_windows",
        "gpio": "desligado",
        "comandos": "janela",
        "angulo_pico_coxa_graus": caminhada.ANGULO_PICO_COXA_GRAUS,
        "angulo_flexao_joelho_graus": caminhada.ANGULO_FLEXAO_JOELHO_GRAUS,
        "duracao_passo_s": DURACAO_PASSO_S,
        "velocidade_referencia_max_rad_s": caminhada.VELOCIDADE_REFERENCIA_MAX_RAD_S,
        "velocidade_referencia_calculada_rad_s": pico_velocidade,
        "kp": caminhada.KP,
        "kd": caminhada.KD,
        "porta": porta,
        "cedencias": [],
        "eventos": [],
        "amostras": [],
        "status": "iniciando",
    }
    print(
        "PASSO PELA JANELA\n"
        "Windows. CANable em %s. GPIO desligado.\n"
        "Segure Esquerda ou Direita. Os dois juntos, ou Em pe, nao movem.\n"
        "A pose lida agora sera segurada em pe.\n"
        "Depois de 70%% da trajetoria pode soltar: o passo termina sozinho.\n"
        "A perna do comando vai a frente e a outra vai para tras.\n"
        "So o joelho da perna acionada flexiona. O da outra fica reto.\n"
        "Ao terminar, esse comando desliga ate a outra perna fechar o ciclo.\n"
        "Soltar torque desliga os motores.\n"
        "coxa=%g° joelho=%g° passo=%.3fs velocidade=%.3f rad/s"
        % (porta, caminhada.ANGULO_PICO_COXA_GRAUS, caminhada.ANGULO_FLEXAO_JOELHO_GRAUS,
           DURACAO_PASSO_S, pico_velocidade),
        flush=True)
    _publicar(estado, rodando=True, pronto=False, fase="ligando",
              titulo="Abrindo %s. GPIO desligado." % porta, erro="")
    link = LinkComReenvio(porta)
    maquina = MaquinaPasso(DURACAO_PASSO_S)
    limite_torque = caminhada.LimiteDeTorque()
    relatorio["usos_de_pico"] = limite_torque.usos
    desde_saturado = {}
    falhas_de_driver = {}
    alertas = {}
    ultima_cedencia = 0.0
    ultima_referencia = {}
    falha_gpio = False
    pausou_no_botao = False
    if led is None:
        led = LedPronto()
    if sinais:
        instalar_parada_por_sinal()
    try:
        led.preparar()
        pisca = {"nivel": 0}

        def avisar_porta(texto):
            print(texto, flush=True)

        def piscar_espera():
            pisca["nivel"] = 0 if pisca["nivel"] else 1
            led.saida(pisca["nivel"])

        abrir_quando_livre(
            link.serial.open, avisar=avisar_porta, enquanto_espera=piscar_espera)
        led.saida(0)
        for comando in (b"C\r", b"M0\r", b"A1\r", b"S8\r", b"O\r"):
            link.write(comando)
            time.sleep(0.06)
            link.poll()
        link.serial.reset_input_buffer()
        link.buffer.clear()
        pedir_feedback(
            link,
            lambda joint: link.write(marcha.frame(joint.can_id, bytes([255] * 7 + [253]))),
            "FD",
            "Sem resposta FD dos IDs %s em %d tentativas")
        agora = time.monotonic()
        origens = {}
        for joint in marcha.JOINTS:
            estado = ler_motor(link, joint.can_id, agora, falhas_de_driver, alertas)
            origens[joint.can_id] = estado["position_rad"]
            baixo, alto = limites[joint.can_id]
            if not baixo <= estado["position_rad"] <= alto:
                print(
                    "AVISO: ID %d na partida em %+.2f°, fora de [%+.2f°, %+.2f°]. "
                    "Seguro essa pose."
                    % (joint.can_id, math.degrees(estado["position_rad"]),
                       math.degrees(baixo), math.degrees(alto)),
                    flush=True)
        pedir_feedback(
            link,
            lambda joint: link.write(
                marcha.frame(joint.can_id, bytes([255] * 7 + [252]))
                + marcha.frame(joint.can_id, caminhada.comando_neutro(joint))),
            "MIT",
            "Sem resposta MIT dos IDs %s em %d tentativas")
        segurar_neutro(link, 12)
        origens_repouso, sem_janela = ler_repouso(link, falhas_de_driver, alertas)
        if sem_janela is not None:
            print(
                "ID %d: janela do encoder ainda invalida. Repito o aquecimento uma vez."
                % sem_janela,
                flush=True)
            segurar_neutro(link, 12)
            origens_repouso, sem_janela = ler_repouso(link, falhas_de_driver, alertas)
            if sem_janela is not None:
                raise RuntimeError(
                    "ID %d: janela do encoder ainda invalida" % sem_janela)
        for can_id, posicao in origens_repouso.items():
            origens[can_id] = posicao
            ultima_referencia[can_id] = 0.0
        limites = ampliar_limites_pela_origem(limites, origens)
        problemas = problemas_de_batente(origens, profile, limites)
        if problemas:
            print(
                "AVISO: o passo sai do batente medido. Sigo com a pose lida: "
                + "; ".join(problemas),
                flush=True)
        relatorio["origens_graus"] = {
            str(can_id): math.degrees(posicao) for can_id, posicao in origens.items()}
        relatorio["status"] = "segurando"
        print("ORIGEM " + " ".join(
            "ID%d=%+.2f°" % (can_id, math.degrees(posicao))
            for can_id, posicao in origens.items()), flush=True)
        print("EM PE. Motores segurando a origem. " + texto_botoes(maquina), flush=True)
        _publicar(estado, pronto=True, fase="em_pe",
                  titulo="EM PE. Motores segurando a origem.",
                  botoes=texto_botoes(maquina))
        led.acender()

        inicio = time.monotonic()
        anterior = inicio
        proximo_tick = inicio
        ultima_amostra = 0.0
        ultimo_aviso = 0.0
        proxima_leitura = 0.0
        ultimo_marco = -1
        while True:
            agora = time.monotonic()
            interromper_se_desligar(pedido)
            if agora < proximo_tick:
                link.poll()
                interromper_se_desligar(pedido)
                if agora >= proxima_leitura:
                    ler_botoes(filtros, agora)
                    proxima_leitura = agora + gpio.AMOSTRA_S
                time.sleep(min(0.001, proximo_tick - agora))
                continue
            atraso_chegada = agora - proximo_tick
            if deve_interromper_por_atraso(
                    atraso_chegada, 0.0, can_acabou_de_voltar(link, agora)):
                raise RuntimeError("Atraso do loop > %.0f ms" % (ATRASO_LOOP_S * 1000))
            inicio_trabalho = time.monotonic()
            link.poll()
            estavel, bruto, leitura_ok, erro_gpio = ler_botoes(filtros, agora)
            interromper_se_desligar(pedido)
            if not leitura_ok:
                if not falha_gpio:
                    print("Falha de leitura dos botoes. Trato os dois como soltos: %s"
                          % erro_gpio, flush=True)
                falha_gpio = True
            else:
                falha_gpio = False
            dt = min(0.04, max(0.0, agora - anterior))
            anterior = agora
            ids_atrasados = []
            for joint in marcha.JOINTS:
                estado = link.states.get(joint.can_id)
                if (estado is None or agora - estado["received_at"] > caminhada.PAUSA_FEEDBACK_S
                        or estado.get("encoder_peak_speed_rad_s") is None):
                    ids_atrasados.append(joint.can_id)
            pausa_feedback = bool(ids_atrasados)
            resultado = maquina.atualizar(estavel, 0.0 if pausa_feedback else dt)
            for evento in resultado["eventos"]:
                relatorio["eventos"].append({"t_parede_s": agora - inicio, "evento": evento})
                print(descrever(evento), flush=True)
                if estado is not None:
                    estado.evento(descrever(evento))
            if resultado["avancando"] and pausou_no_botao:
                pausou_no_botao = False
                print("RETOMA %s em t=%.2fs." % (maquina.ativa, resultado["t"]), flush=True)
            elif (maquina.ativa is not None and estavel[maquina.ativa] == 0
                    and not maquina.segue_sozinho
                    and not pausou_no_botao and resultado["t"] > 0.0):
                pausou_no_botao = True
                print("PAUSA %s em t=%.2fs. Motores segurando essa pose."
                      % (maquina.ativa, resultado["t"]), flush=True)

            ceder = False
            motivo = None
            pacote = bytearray()
            fotos = []
            for joint in marcha.JOINTS:
                estado = ler_motor(link, joint.can_id, agora, falhas_de_driver, alertas)
                baixo, alto = limites[joint.can_id]
                folga = math.radians(FOLGA_BATENTE_GRAUS)
                fora = not (baixo - folga <= estado["position_rad"] <= alto + folga)
                if confirmar_leitura(alertas, ("batente", joint.can_id), fora):
                    raise RuntimeError(
                        "ID %d: posicao saiu do batente (%+.2f°) em %d leituras seguidas"
                        % (joint.can_id, math.degrees(estado["position_rad"]),
                           LEITURAS_PARA_CONFIRMAR))
                if resultado["perna"] is None:
                    ref = 0.0
                else:
                    ref = referencia_da_perna(
                        joint, resultado["t"], resultado["perna"], profile)
                alvo = origens[joint.can_id] + math.radians(ref)
                if not baixo <= alvo <= alto:
                    alvo = min(alto, max(baixo, alvo))
                    if resultado["avancando"]:
                        ceder = True
                        motivo = "ID %d: referencia fora do batente" % joint.can_id
                erro = alvo - estado["position_rad"]
                taxa = 0.0 if dt <= 0.0 else math.radians(
                    ref - ultima_referencia[joint.can_id]) / dt
                taxa = max(-caminhada.VELOCIDADE_REFERENCIA_MAX_RAD_S,
                           min(caminhada.VELOCIDADE_REFERENCIA_MAX_RAD_S, taxa))
                ultima_referencia[joint.can_id] = ref
                demanda_nm = caminhada.KP * abs(erro) + caminhada.KD * abs(taxa)
                teto = limite_torque.teto_nm(joint.can_id, agora, demanda_nm)
                alcance = caminhada.lead_rad(teto, taxa)
                comando = estado["position_rad"] + max(-alcance, min(alcance, erro))
                comando = caminhada.inward_quantized_target(
                    joint.can_id, comando, math.degrees(baixo), math.degrees(alto))
                comando = min(alto, max(baixo, comando))
                pacote.extend(marcha.frame(
                    joint.can_id, caminhada.comando_mit(joint, comando, taxa)))
                erro_graus = math.degrees(erro)
                no_teto = (abs(math.degrees(comando - estado["position_rad"]))
                           >= math.degrees(alcance) - 0.05
                           and abs(erro_graus) > caminhada.ERRO_PARA_CEDER_GRAUS)
                medido_no_teto = abs(estado["effort_nm"]) >= caminhada.FRACA_DE_TORQUE * teto
                saturado = no_teto or (
                    medido_no_teto and abs(erro_graus) > caminhada.ERRO_PARA_CEDER_GRAUS)
                if not resultado["avancando"] or not saturado:
                    desde_saturado.pop(joint.can_id, None)
                else:
                    desde, posicao_inicial = desde_saturado.setdefault(
                        joint.can_id, (agora, estado["position_rad"]))
                    sentido = 1.0 if erro > 0.0 else -1.0
                    avancou = (estado["position_rad"] - posicao_inicial) * sentido > math.radians(
                        caminhada.AVANCO_MINIMO_GRAUS)
                    if avancou:
                        desde_saturado[joint.can_id] = (agora, estado["position_rad"])
                    elif (agora - desde >= caminhada.TEMPO_SEM_PROGRESSO_S
                            and agora - ultima_cedencia >= caminhada.FOLGA_ENTRE_CEDENCIAS_S):
                        ceder = True
                        motivo = (
                            "ID %d: teto vigente %.1f Nm sem concluir o movimento "
                            "(esforco %+.2f Nm, erro %+.2f°)"
                            % (joint.can_id, teto, estado["effort_nm"], erro_graus))
                fotos.append({
                    "id": joint.can_id,
                    "referencia_graus": ref,
                    "atual_graus": math.degrees(estado["position_rad"] - origens[joint.can_id]),
                    "esforco_nm": estado["effort_nm"],
                    "teto_nm": teto,
                    "pico_momentaneo": teto > caminhada.TORQUE_NOMINAL_NM[joint.can_id],
                    "temperatura_c": estado["temperature_c"],
                    "erro_graus": erro_graus,
                })
            link.write(bytes(pacote))
            if resultado["avancando"]:
                fase = "passo"
            elif maquina.ativa is None:
                fase = "em_pe"
            else:
                fase = "pausa"
            _publicar(
                estado,
                fase=fase,
                botoes=texto_botoes(maquina),
                juntas=[{
                    "id": foto["id"],
                    "graus": foto["atual_graus"],
                    "esforco_nm": foto["esforco_nm"],
                    "temperatura_c": foto["temperatura_c"],
                } for foto in fotos],
            )
            if ceder and motivo and resultado["avancando"] and maquina.ativa is not None:
                ultima_cedencia = agora
                desde_saturado.clear()
                maquina.abortar(estavel)
                pausou_no_botao = False
                intensidade = max(
                    caminhada.INTENSIDADE_MINIMA,
                    intensidade * caminhada.FATOR_REDUCAO_INTENSIDADE)
                caminhada.aplicar_intensidade(meta, intensidade)
                print(
                    "CEDE passo cancelado sem contar. Volto a referencia em pe. "
                    "Solte o botao e aperte de novo para tentar. "
                    "intensidade=%.2f coxa=%.2f° joelho=%.2f° | %s"
                    % (intensidade, meta["requested_hip_peak_deg"],
                       meta["knee_flexion_deg"], motivo),
                    flush=True)
                relatorio["cedencias"].append({
                    "t_passo_s": resultado["t"],
                    "t_parede_s": agora - inicio,
                    "fase": "aborta_passo",
                    "motivo": motivo,
                    "intensidade": intensidade,
                    "coxa_graus": meta["requested_hip_peak_deg"],
                    "joelho_graus": meta["knee_flexion_deg"],
                })
            if agora - ultima_amostra >= 0.5 or resultado["eventos"] or ceder:
                relatorio["amostras"].append({
                    "t_passo_s": resultado["t"],
                    "t_parede_s": agora - inicio,
                    "perna": resultado["perna"],
                    "avancando": resultado["avancando"],
                    "pino31": bruto["esquerda"],
                    "pino33": bruto["direita"],
                    "estavel31": estavel["esquerda"],
                    "estavel33": estavel["direita"],
                    "intensidade": intensidade,
                    "pausa_feedback": pausa_feedback,
                    "ciclos_fechados": maquina.ciclos,
                    "juntas": fotos,
                })
                ultima_amostra = agora
            if resultado["avancando"]:
                marco = int(resultado["t"] // 0.5)
                if marco != ultimo_marco:
                    ultimo_marco = marco
                    print(
                        "t=%.2fs %s " % (resultado["t"], resultado["perna"])
                        + " ".join(
                            "ID%d=%+.2f°/%+.1fNm" % (
                                foto["id"], foto["atual_graus"], foto["esforco_nm"])
                            for foto in fotos),
                        flush=True)
            elif agora - ultimo_aviso >= 2.0 and not resultado["eventos"]:
                ultimo_aviso = agora
                if maquina.ativa is None:
                    titulo = "EM PE ciclos=%d" % maquina.ciclos
                elif estavel[maquina.ativa] == 0:
                    titulo = "PAUSA %s t=%.2fs" % (maquina.ativa, resultado["t"])
                else:
                    titulo = "SEGURA %s t=%.2fs" % (maquina.ativa, resultado["t"])
                print(
                    "%s %s bruto=%d/%d estavel=%d/%d "
                    % (titulo, texto_botoes(maquina),
                       bruto["esquerda"], bruto["direita"],
                       estavel["esquerda"], estavel["direita"])
                    + " ".join(
                        "ID%d=%+.2f°/%+.1fNm" % (
                            foto["id"], foto["atual_graus"], foto["esforco_nm"])
                        for foto in fotos),
                    flush=True)
            fim = time.monotonic()
            duracao = fim - inicio_trabalho
            if deve_interromper_por_atraso(
                    0.0, duracao, can_acabou_de_voltar(link, fim)):
                raise RuntimeError("Atraso do loop > %.0f ms" % (ATRASO_LOOP_S * 1000))
            if (atraso_chegada > caminhada.PERIODO_S or duracao > caminhada.PERIODO_S
                    or can_acabou_de_voltar(link, fim)):
                proximo_tick = fim + caminhada.PERIODO_S
            else:
                proximo_tick += caminhada.PERIODO_S
    except KeyboardInterrupt as exc:
        relatorio["status"] = "interrompida"
        signo = exc.args[0] if exc.args else None
        if pedido is not None and pedido.desligar:
            relatorio["motivo"] = "soltar torque pela janela"
            print("INTERROMPIDA: a janela pediu para soltar o torque.", flush=True)
            _publicar(estado, fase="parado", pronto=False,
                      titulo="Torque solto pela janela.")
        else:
            relatorio["motivo"] = "parada solicitada"
            print("INTERROMPIDA: parada solicitada. Desligando o torque.", flush=True)
            _publicar(estado, fase="parado", pronto=False,
                      titulo="Parada solicitada. Torque solto.")
        if signo == signal.SIGTERM and not (pedido is not None and pedido.desligar):
            raise
    except Exception as exc:
        relatorio["status"] = "interrompida"
        relatorio["motivo"] = str(exc)
        print("INTERROMPIDA:", exc, flush=True)
        _publicar(estado, fase="erro", pronto=False, erro=str(exc),
                  titulo="Interrompida: %s" % exc)
    finally:
        led.apagar()
        _publicar(estado, rodando=False, pronto=False)
        relatorio["ciclos_fechados"] = maquina.ciclos
        if link.serial.is_open:
            relatorio["erros_desligamento"] = link.shutdown()
            relatorio["fd_enviado"] = True
        else:
            relatorio["erros_desligamento"] = []
            relatorio["fd_enviado"] = False
        relatorio["torque_desligado_confirmado"] = False
        destino = pasta / "execucao.json"
        destino.write_text(
            json.dumps(relatorio, indent=2, ensure_ascii=False), encoding="utf-8")
        print("REGISTRO", destino, "STATUS", relatorio["status"], flush=True)
    return relatorio["status"] == "interrompida" and relatorio.get("motivo") == "parada solicitada"


def instalar_parada_por_sinal():
    """SIGINT e SIGTERM caem no finally e mandam o quadro de desligar."""

    def pedir_parada(signum, _frame):
        raise KeyboardInterrupt(signum)

    signal.signal(signal.SIGINT, pedir_parada)
    signal.signal(signal.SIGTERM, pedir_parada)


if __name__ == "__main__":
    if "--checar" in sys.argv:
        pico = checar_logica()
        gpio.checar_logica()
        print(
            "Logica ok. Um passo dura %.3f s, coxa %g°, joelho %g°, "
            "velocidade de referencia %.3f rad/s. CAN nao foi aberta."
            % (DURACAO_PASSO_S, caminhada.ANGULO_PICO_COXA_GRAUS,
               caminhada.ANGULO_FLEXAO_JOELHO_GRAUS, pico),
            flush=True)
        sys.exit(0)
    import interface_exo
    sys.exit(interface_exo.main())
