"""Caminhada da figura com torque nominal permanente e pico só momentâneo.

O comando fica no torque nominal de cada motor. O pico publicado só entra
por uma janela curta, quando o nominal não fecha o movimento, e depois
fica bloqueado. Não pode ser usado de forma permanente. Se nem o pico
curto fecha o movimento, a fase comum avança e a amplitude desce.
Falha, temperatura, velocidade, batente ou perda de feedback ainda param.

Versão para Raspberry Pi 4. A CANable v2 entra em /dev/ttyACM0.
A origem de cada junta é a leitura no início deste ensaio.
"""
import glob
import json
import math
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import suporte_marcha as marcha
from limites_operacionais_exo import inward_quantized_target, operational_limits_deg


# --- Ângulos da mesma caminhada -------------------------------------------
ANGULO_PICO_COXA_GRAUS = 18.0
ANGULO_FLEXAO_JOELHO_GRAUS = 45.0
# 8 s era o ciclo anterior. Dividir por 0,7 deixa cada passo 30% mais lento.
DURACAO_CICLO_S = 8.0 / 0.7
PASSOS_ALTERNADOS = 5  #5
# Total = DURACAO_CICLO_S * (PASSOS_ALTERNADOS + 1) / 2.
# Se o ciclo ficar curto demais, a curva passa de VELOCIDADE_REFERENCIA_MAX_RAD_S
# e o programa não move o exo.

# --- Velocidade ------------------------------------------------------------
# Teto da referência. O passo a 45° neste ciclo fica perto de 1,20 rad/s.
# O corte de segurança continua em VELOCIDADE_CORTE_RAD_S.
VELOCIDADE_REFERENCIA_MAX_RAD_S = 1.75
VELOCIDADE_CORTE_RAD_S = 3.0

# --- Torque ---------------------------------------------------------------
# O nominal de comando é o teto permanente. O pico é momentâneo.
# Publicado: AK80-64 (ID1 e ID3) 48 Nm nominais e 120 Nm de pico;
# AK80-8 KV60 (ID2 e ID4) 10 Nm nominais e 25 Nm de pico.
# O comando permanente usa 50% acima do nominal publicado.
# A escala MIT (144 Nm e 32 Nm) só cabe no quadro CAN; o comando para no pico.
TORQUE_NOMINAL_PUBLICADO_NM = {1: 48.0, 2: 10.0, 3: 48.0, 4: 10.0}
FATOR_TORQUE_NOMINAL = 1.5
TORQUE_NOMINAL_NM = {
    can_id: valor * FATOR_TORQUE_NOMINAL
    for can_id, valor in TORQUE_NOMINAL_PUBLICADO_NM.items()}
TORQUE_PICO_NM = {1: 120.0, 2: 25.0, 3: 120.0, 4: 25.0}
# Acima do nominal, no máximo este intervalo. Depois o pico fica proibido
# e o comando volta ao nominal. Isso impede uso permanente do pico.
DURACAO_MAXIMA_PICO_S = 0.25
REPOUSO_APOS_PICO_S = 2.0

KP = 500.0
KD = 4.5
PERIODO_S = 0.02
# CANable v2 na Raspberry Pi 4. Confira com: ls /dev/ttyACM*
PORTA_PADRAO = "/dev/ttyACM0"
TEMPERATURA_CORTE_C = 70.0
WATCHDOG_S = 0.12
PAUSA_FEEDBACK_S = 0.04
ERRO_PARA_CEDER_GRAUS = 1.5
AVANCO_MINIMO_GRAUS = 0.15
TEMPO_SEM_PROGRESSO_S = 0.45
FOLGA_ENTRE_CEDENCIAS_S = 0.35
FRACA_DE_TORQUE = 0.92
FATOR_REDUCAO_INTENSIDADE = 0.80
INTENSIDADE_MINIMA = 0.35
ESFORCO_REPOUSO_MAX_NM = 3.0
MARGEM_BATENTE_GRAUS = 1.0


def perfil_da_caminhada():
    """Carrega a curva da figura e grava os ângulos usados neste ensaio."""
    times, curve, meta = marcha.load_figure_profile(DURACAO_CICLO_S, 28.0)
    meta["requested_hip_peak_deg"] = ANGULO_PICO_COXA_GRAUS
    meta["knee_flexion_deg"] = ANGULO_FLEXAO_JOELHO_GRAUS
    meta["knee_swing_flexion_hold_fraction"] = marcha.KNEE_SWING_FLEXION_HOLD_FRACTION
    return (times, curve, meta)


def aplicar_intensidade(meta, intensidade):
    meta["requested_hip_peak_deg"] = ANGULO_PICO_COXA_GRAUS * intensidade
    meta["knee_flexion_deg"] = ANGULO_FLEXAO_JOELHO_GRAUS * intensidade


def referencia_graus(joint, t, profile):
    return marcha.figure_steps_reference_deg(
        joint, t, PASSOS_ALTERNADOS, "ambas", profile)


def duracoes():
    caminhada = DURACAO_CICLO_S * PASSOS_ALTERNADOS / 2
    return caminhada, DURACAO_CICLO_S / 2, caminhada + DURACAO_CICLO_S / 2


def pico_velocidade_referencia_rad_s(profile):
    _, _, total = duracoes()
    passo = PERIODO_S
    amostras = int(round(total / passo))
    pico = 0.0
    anterior = {j.can_id: referencia_graus(j, 0.0, profile) for j in marcha.JOINTS}
    for indice in range(1, amostras + 1):
        t = min(total, indice * passo)
        for joint in marcha.JOINTS:
            atual = referencia_graus(joint, t, profile)
            pico = max(pico, abs(math.radians(atual - anterior[joint.can_id]) / passo))
            anterior[joint.can_id] = atual
    return pico


def limites_rad():
    limites = {}
    for joint in marcha.JOINTS:
        baixo, alto = marcha.MEASURED_BATENT_LIMITS_DEG[joint.can_id]
        seguro_baixo, seguro_alto = operational_limits_deg(
            joint.can_id, baixo, alto, MARGEM_BATENTE_GRAUS)
        limites[joint.can_id] = (math.radians(seguro_baixo), math.radians(seguro_alto))
    return limites


def envelope_cabe(origens, profile, limites):
    """Recusa a partida se a curva inteira, na intensidade 1, sai do batente."""
    _, _, total = duracoes()
    problemas = []
    for joint in marcha.JOINTS:
        baixo, alto = limites[joint.can_id]
        amostras = [referencia_graus(joint, i * PERIODO_S, profile)
                    for i in range(int(total / PERIODO_S) + 1)]
        for graus in (min(amostras), max(amostras)):
            alvo = origens[joint.can_id] + math.radians(graus)
            if not baixo <= alvo <= alto:
                problemas.append(
                    f"ID {joint.can_id}: alvo {math.degrees(alvo):+.2f}° fora de "
                    f"[{math.degrees(baixo):+.2f}°, {math.degrees(alto):+.2f}°]")
    return problemas


def avance_da_fase(t, caminhada_s):
    meio = DURACAO_CICLO_S / 2
    if t >= caminhada_s:
        return t
    proximo = (math.floor(t / meio) + 1) * meio
    return caminhada_s if proximo >= caminhada_s - 1e-9 else proximo


class LimiteDeTorque:
    """Nominal o tempo todo. Pico só numa janela curta, seguido de repouso."""

    def __init__(self):
        self.inicio_pico = {}
        self.bloqueado_ate = {}
        self.usos = []

    def teto_nm(self, can_id, agora, demanda_nm):
        nominal = TORQUE_NOMINAL_NM[can_id]
        pico = TORQUE_PICO_NM[can_id]
        if demanda_nm <= nominal:
            if can_id in self.inicio_pico:
                self._encerrar(can_id, agora, "demanda voltou ao nominal")
            return nominal
        if agora < self.bloqueado_ate.get(can_id, 0.0):
            return nominal
        inicio = self.inicio_pico.setdefault(can_id, agora)
        if agora - inicio <= DURACAO_MAXIMA_PICO_S:
            return pico
        self._encerrar(can_id, agora, "janela de pico esgotada")
        return nominal

    def _encerrar(self, can_id, agora, motivo):
        inicio = self.inicio_pico.pop(can_id, None)
        if inicio is None:
            return
        self.bloqueado_ate[can_id] = agora + REPOUSO_APOS_PICO_S
        self.usos.append({
            "id": can_id,
            "inicio_s": inicio,
            "fim_s": agora,
            "duracao_s": agora - inicio,
            "motivo": motivo,
            "nominal_nm": TORQUE_NOMINAL_NM[can_id],
            "pico_nm": TORQUE_PICO_NM[can_id],
        })


def checar_logica():
    """Confere a troca de movimento e o limite de pico sem abrir a CAN."""
    meio = DURACAO_CICLO_S / 2
    assert math.isclose(avance_da_fase(meio * 0.4, meio * 10), meio)
    assert math.isclose(avance_da_fase(meio * 10 - 0.1, meio * 10), meio * 10)
    assert math.isclose(avance_da_fase(meio * 10, meio * 10), meio * 10)
    for can_id, publicado in TORQUE_NOMINAL_PUBLICADO_NM.items():
        assert math.isclose(TORQUE_NOMINAL_NM[can_id], publicado * FATOR_TORQUE_NOMINAL)
        assert TORQUE_NOMINAL_NM[can_id] < TORQUE_PICO_NM[can_id]
    limite = LimiteDeTorque()
    acima_do_nominal = TORQUE_NOMINAL_NM[2] + 1.0
    assert limite.teto_nm(2, 0.0, 4.0) == TORQUE_NOMINAL_NM[2]
    assert limite.teto_nm(2, 0.0, acima_do_nominal) == TORQUE_PICO_NM[2]
    assert limite.teto_nm(2, DURACAO_MAXIMA_PICO_S, acima_do_nominal) == TORQUE_PICO_NM[2]
    assert limite.teto_nm(2, DURACAO_MAXIMA_PICO_S + 0.01, acima_do_nominal) == TORQUE_NOMINAL_NM[2]
    assert limite.teto_nm(2, DURACAO_MAXIMA_PICO_S + REPOUSO_APOS_PICO_S, acima_do_nominal) == TORQUE_NOMINAL_NM[2]
    assert limite.teto_nm(2, DURACAO_MAXIMA_PICO_S + REPOUSO_APOS_PICO_S + 0.02, acima_do_nominal) == TORQUE_PICO_NM[2]
    assert limite.teto_nm(1, 0.0, TORQUE_NOMINAL_NM[1] - 1.0) == TORQUE_NOMINAL_NM[1]
    assert limite.teto_nm(1, 0.0, TORQUE_NOMINAL_NM[1] + 1.0) == TORQUE_PICO_NM[1]
    continuo = LimiteDeTorque()
    instante = 0.0
    maior_sequencia = 0.0
    sequencia = 0.0
    while instante <= 5.0:
        teto = continuo.teto_nm(4, instante, TORQUE_PICO_NM[4])
        if teto > TORQUE_NOMINAL_NM[4]:
            sequencia += 0.02
            maior_sequencia = max(maior_sequencia, sequencia)
        else:
            sequencia = 0.0
        instante += 0.02
    if maior_sequencia > DURACAO_MAXIMA_PICO_S + 0.03:
        raise RuntimeError("O pico permaneceu acima do tempo momentâneo permitido")
    intensidade = 1.0
    intensidade = max(INTENSIDADE_MINIMA, intensidade * FATOR_REDUCAO_INTENSIDADE)
    assert math.isclose(intensidade, 0.80)
    for _ in range(20):
        intensidade = max(INTENSIDADE_MINIMA, intensidade * FATOR_REDUCAO_INTENSIDADE)
    assert math.isclose(intensidade, INTENSIDADE_MINIMA)
    profile = perfil_da_caminhada()
    pico = pico_velocidade_referencia_rad_s(profile)
    if pico > VELOCIDADE_REFERENCIA_MAX_RAD_S + 1e-6:
        raise RuntimeError(
            f"Velocidade de referência {pico:.3f} rad/s passa de "
            f"{VELOCIDADE_REFERENCIA_MAX_RAD_S:g} rad/s")
    return pico


def lead_rad(teto_nm, velocidade_comando):
    """Erro de posição que produz, com Kp, no máximo o teto de torque vigente."""
    restante = teto_nm - KD * abs(velocidade_comando)
    if restante <= 0:
        return 0.0
    return restante / KP


def comando_mit(joint, posicao, velocidade):
    return marcha.encode_command(
        posicao, joint, ff=0.0, kp=KP, kd=KD, velocity=velocidade)


def comando_neutro(joint):
    """MIT com Kp zero: habilita o modo sem puxar o encoder para um alvo."""
    return marcha.encode_command(0.0, joint, ff=0.0, kp=0.0, kd=0.0, velocity=0.0)


def ler_estado(link, can_id, agora, atraso_max_s=None, recusar_falha=True,
               recusar_velocidade=True, recusar_temperatura=True):
    limite = WATCHDOG_S if atraso_max_s is None else atraso_max_s
    estado = link.states.get(can_id)
    if estado is None or agora - estado["received_at"] > limite:
        raise RuntimeError(f"ID {can_id}: feedback atrasado > {limite * 1000:.0f} ms")
    if recusar_falha and estado["fault"]:
        raise RuntimeError(f"ID {can_id}: falha do driver {estado['fault']}")
    if recusar_temperatura and estado["temperature_c"] >= TEMPERATURA_CORTE_C:
        raise RuntimeError(
            f"ID {can_id}: temperatura {estado['temperature_c']} °C")
    bruto = abs(estado["velocity_rad_s"])
    estimado = estado.get("encoder_peak_speed_rad_s")
    if recusar_velocidade and estimado is not None and estimado > VELOCIDADE_CORTE_RAD_S:
        raise RuntimeError(f"ID {can_id}: velocidade {estimado:.2f} rad/s")
    if recusar_velocidade and bruto > marcha.RAW_SPEED_HARD_TRIP_RAD_S:
        raise RuntimeError(f"ID {can_id}: velocidade bruta {bruto:.2f} rad/s")
    return estado


def resolver_porta(argumento=None):
    """Usa o argumento, CANABLE_PORT, /dev/ttyACM0 ou a única ttyACM achada."""
    if argumento:
        porta = argumento
    elif os.environ.get("CANABLE_PORT"):
        porta = os.environ["CANABLE_PORT"]
    else:
        porta = PORTA_PADRAO
        if not os.path.exists(porta):
            achados = sorted(glob.glob("/dev/ttyACM*"))
            if len(achados) == 1:
                porta = achados[0]
            elif not achados:
                raise RuntimeError(
                    "CANable nao encontrada. Ligue o adaptador e rode ls /dev/ttyACM*")
            else:
                raise RuntimeError(
                    "Ha mais de uma porta ACM: %s. Passe o nome na linha de comando."
                    % ", ".join(achados))
    if not os.path.exists(porta) and not porta.upper().startswith("COM"):
        raise RuntimeError("Porta inexistente: %s. Confira com ls /dev/ttyACM*" % porta)
    return porta


def executar(porta=None):
    pico_velocidade = checar_logica()
    porta = resolver_porta(porta)
    caminhada_s, retorno_s, total_s = duracoes()
    profile = perfil_da_caminhada()
    meta = profile[2]
    intensidade = 1.0
    aplicar_intensidade(meta, intensidade)
    limites = limites_rad()
    pasta = Path(__file__).resolve().parent / "ensaios" / (
        "caminhada_torque_pleno_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    pasta.mkdir(parents=True, exist_ok=True)
    relatorio = {
        "angulo_pico_coxa_graus": ANGULO_PICO_COXA_GRAUS,
        "angulo_flexao_joelho_graus": ANGULO_FLEXAO_JOELHO_GRAUS,
        "duracao_ciclo_s": DURACAO_CICLO_S,
        "passos": PASSOS_ALTERNADOS,
        "velocidade_referencia_max_rad_s": VELOCIDADE_REFERENCIA_MAX_RAD_S,
        "velocidade_referencia_calculada_rad_s": pico_velocidade,
        "torque_nominal_publicado_nm": {str(k): v for k, v in TORQUE_NOMINAL_PUBLICADO_NM.items()},
        "fator_torque_nominal": FATOR_TORQUE_NOMINAL,
        "torque_nominal_nm": {str(k): v for k, v in TORQUE_NOMINAL_NM.items()},
        "torque_pico_nm": {str(k): v for k, v in TORQUE_PICO_NM.items()},
        "duracao_maxima_pico_s": DURACAO_MAXIMA_PICO_S,
        "repouso_apos_pico_s": REPOUSO_APOS_PICO_S,
        "pico_e_momentaneo": True,
        "kp": KP,
        "kd": KD,
        "porta": porta,
        "cedencias": [],
        "amostras": [],
        "status": "iniciando",
    }
    print(
        "VARIAVEIS "
        f"coxa={ANGULO_PICO_COXA_GRAUS:g}° "
        f"joelho={ANGULO_FLEXAO_JOELHO_GRAUS:g}° "
        f"ciclo={DURACAO_CICLO_S:g}s passos={PASSOS_ALTERNADOS} "
        f"velocidade_max={VELOCIDADE_REFERENCIA_MAX_RAD_S:g} rad/s "
        f"calculada={pico_velocidade:.3f} rad/s "
        f"nominal_Nm={TORQUE_NOMINAL_NM} pico_Nm={TORQUE_PICO_NM} "
        f"pico_max={DURACAO_MAXIMA_PICO_S:g}s repouso={REPOUSO_APOS_PICO_S:g}s",
        flush=True)
    link = marcha.Link(porta)
    origens = {}
    segurar_retorno = {}
    desde_saturado = {}
    ultima_cedencia = 0.0
    ultima_posicao = {}
    ultimo_comando = {}
    ultima_referencia = {}
    limite_torque = LimiteDeTorque()
    relatorio["usos_de_pico"] = limite_torque.usos
    t_trajetoria = 0.0
    pausa_feedback = False
    try:
        link.serial.open()
        for comando in (b"C\r", b"M0\r", b"A1\r", b"S8\r", b"O\r"):
            link.write(comando)
            time.sleep(0.06)
            link.poll()
        link.serial.reset_input_buffer()
        link.buffer.clear()
        faltando, tentativas = marcha.request_fresh_feedback(
            link, marcha.JOINTS, {j.can_id for j in marcha.JOINTS},
            lambda j: link.write(marcha.frame(j.can_id, bytes([255] * 7 + [253]))),
            phase="FD")
        if faltando:
            raise RuntimeError(f"Sem resposta FD dos IDs {sorted(faltando)} em {tentativas} tentativas")
        agora = time.monotonic()
        for joint in marcha.JOINTS:
            estado = ler_estado(link, joint.can_id, agora)
            origens[joint.can_id] = estado["position_rad"]
            baixo, alto = limites[joint.can_id]
            if not baixo <= estado["position_rad"] <= alto:
                raise RuntimeError(
                    f"ID {joint.can_id}: posição {math.degrees(estado['position_rad']):+.2f}° "
                    "fora da margem do batente")
        problemas = envelope_cabe(origens, profile, limites)
        if problemas:
            raise RuntimeError("Curva não cabe nos batentes a partir da posição atual: " + "; ".join(problemas))
        faltando, _ = marcha.request_fresh_feedback(
            link, marcha.JOINTS, {j.can_id for j in marcha.JOINTS},
            lambda j: link.write(
                marcha.frame(j.can_id, bytes([255] * 7 + [252]))
                + marcha.frame(j.can_id, comando_neutro(j))),
            phase="MIT_Kp0")
        if faltando:
            raise RuntimeError(f"Sem resposta MIT dos IDs {sorted(faltando)}")
        for _ in range(12):
            link.write(b"".join(
                marcha.frame(j.can_id, comando_neutro(j)) for j in marcha.JOINTS))
            prazo = time.monotonic() + PERIODO_S
            while time.monotonic() < prazo:
                link.poll()
                time.sleep(0.001)
        agora = time.monotonic()
        for joint in marcha.JOINTS:
            estado = ler_estado(link, joint.can_id, agora)
            if abs(estado["effort_nm"]) >= ESFORCO_REPOUSO_MAX_NM:
                raise RuntimeError(
                    f"ID {joint.can_id}: esforço em repouso {estado['effort_nm']:+.2f} Nm")
            if estado.get("encoder_peak_speed_rad_s") is None:
                raise RuntimeError(f"ID {joint.can_id}: janela do encoder ainda inválida")
            origens[joint.can_id] = estado["position_rad"]
            ultimo_comando[joint.can_id] = estado["position_rad"]
            ultima_posicao[joint.can_id] = estado["position_rad"]
            ultima_referencia[joint.can_id] = 0.0
        problemas = envelope_cabe(origens, profile, limites)
        if problemas:
            raise RuntimeError(
                "Curva não cabe nos batentes a partir da posição atual: "
                + "; ".join(problemas))
        relatorio["origens_graus"] = {
            str(i): math.degrees(p) for i, p in origens.items()}
        print("ORIGEM " + " ".join(
            f"ID{i}={math.degrees(p):+.2f}°" for i, p in origens.items()), flush=True)

        inicio = time.monotonic()
        anterior = inicio
        proximo_tick = inicio
        ultimo_print = -1
        ultima_amostra = 0.0
        while True:
            agora = time.monotonic()
            if agora < proximo_tick:
                link.poll()
                time.sleep(min(0.001, proximo_tick - agora))
                continue
            if agora - proximo_tick > 0.06:
                raise RuntimeError("Atraso do loop > 60 ms")
            link.poll()
            dt = min(0.04, max(0.0, agora - anterior))
            anterior = agora
            ids_atrasados = []
            for joint in marcha.JOINTS:
                estado = link.states.get(joint.can_id)
                if (estado is None or agora - estado["received_at"] > PAUSA_FEEDBACK_S
                        or estado.get("encoder_peak_speed_rad_s") is None):
                    ids_atrasados.append(joint.can_id)
            pausa_feedback = bool(ids_atrasados)
            if not pausa_feedback:
                t_trajetoria += dt
            t = min(t_trajetoria, total_s + 2.0)
            if agora - inicio > 3 * (total_s + 4):
                raise RuntimeError("Tempo de parede esgotado")

            ceder = False
            motivo = None
            pacote = bytearray()
            fotos = []
            for joint in marcha.JOINTS:
                estado = ler_estado(link, joint.can_id, agora)
                baixo, alto = limites[joint.can_id]
                if not baixo - math.radians(0.2) <= estado["position_rad"] <= alto + math.radians(0.2):
                    raise RuntimeError(
                        f"ID {joint.can_id}: posição saiu do batente "
                        f"({math.degrees(estado['position_rad']):+.2f}°)")
                if joint.can_id in segurar_retorno:
                    ref = segurar_retorno[joint.can_id]
                elif t >= caminhada_s:
                    ref = 0.0
                else:
                    ref = referencia_graus(joint, t, profile)
                alvo = origens[joint.can_id] + math.radians(ref)
                if not baixo <= alvo <= alto:
                    ceder = True
                    motivo = f"ID {joint.can_id}: referência fora do batente"
                    alvo = min(alto, max(baixo, alvo))
                erro = alvo - estado["position_rad"]
                taxa = 0.0 if dt <= 0 else math.radians(ref - ultima_referencia[joint.can_id]) / dt
                taxa = max(-VELOCIDADE_REFERENCIA_MAX_RAD_S,
                           min(VELOCIDADE_REFERENCIA_MAX_RAD_S, taxa))
                ultima_referencia[joint.can_id] = ref
                demanda_nm = KP * abs(erro) + KD * abs(taxa)
                teto = limite_torque.teto_nm(joint.can_id, agora, demanda_nm)
                alcance = lead_rad(teto, taxa)
                comando = estado["position_rad"] + max(-alcance, min(alcance, erro))
                comando = inward_quantized_target(
                    joint.can_id, comando, math.degrees(baixo), math.degrees(alto))
                comando = min(alto, max(baixo, comando))
                ultimo_comando[joint.can_id] = comando
                pacote.extend(marcha.frame(
                    joint.can_id, comando_mit(joint, comando, taxa)))
                erro_graus = math.degrees(erro)
                no_teto = (abs(math.degrees(comando - estado["position_rad"])) >=
                           math.degrees(alcance) - 0.05
                           and abs(erro_graus) > ERRO_PARA_CEDER_GRAUS)
                medido_no_teto = abs(estado["effort_nm"]) >= FRACA_DE_TORQUE * teto
                saturado = no_teto or (medido_no_teto and abs(erro_graus) > ERRO_PARA_CEDER_GRAUS)
                if not saturado:
                    desde_saturado.pop(joint.can_id, None)
                else:
                    desde, posicao_inicial = desde_saturado.setdefault(
                        joint.can_id, (agora, estado["position_rad"]))
                    sentido = 1.0 if erro > 0 else -1.0
                    avancou = (estado["position_rad"] - posicao_inicial) * sentido > math.radians(
                        AVANCO_MINIMO_GRAUS)
                    if avancou:
                        desde_saturado[joint.can_id] = (agora, estado["position_rad"])
                    elif (agora - desde >= TEMPO_SEM_PROGRESSO_S
                            and agora - ultima_cedencia >= FOLGA_ENTRE_CEDENCIAS_S):
                        ceder = True
                        motivo = (
                            f"ID {joint.can_id}: teto vigente {teto:.1f} Nm sem concluir "
                            f"o movimento (esforço {estado['effort_nm']:+.2f} Nm, "
                            f"erro {erro_graus:+.2f}°)")
                fotos.append({
                    "id": joint.can_id,
                    "referencia_graus": ref,
                    "atual_graus": math.degrees(estado["position_rad"] - origens[joint.can_id]),
                    "esforco_nm": estado["effort_nm"],
                    "teto_nm": teto,
                    "pico_momentaneo": teto > TORQUE_NOMINAL_NM[joint.can_id],
                    "temperatura_c": estado["temperature_c"],
                    "erro_graus": erro_graus,
                })
            link.write(bytes(pacote))
            if ceder and motivo:
                ultima_cedencia = agora
                desde_saturado.clear()
                if t >= caminhada_s:
                    for foto in fotos:
                        if abs(foto["erro_graus"]) > ERRO_PARA_CEDER_GRAUS:
                            segurar_retorno[foto["id"]] = foto["atual_graus"]
                    fase = "retorno"
                else:
                    t_trajetoria = avance_da_fase(t, caminhada_s)
                    intensidade = max(INTENSIDADE_MINIMA,
                                      intensidade * FATOR_REDUCAO_INTENSIDADE)
                    aplicar_intensidade(meta, intensidade)
                    fase = "passo"
                evento = {
                    "t_s": t, "fase": fase, "motivo": motivo,
                    "intensidade": intensidade,
                    "coxa_graus": meta["requested_hip_peak_deg"],
                    "joelho_graus": meta["knee_flexion_deg"],
                }
                relatorio["cedencias"].append(evento)
                print(
                    f"CEDE {fase} t={t:.2f}s intensidade={intensidade:.2f} "
                    f"coxa={meta['requested_hip_peak_deg']:.2f}° "
                    f"joelho={meta['knee_flexion_deg']:.2f}° | {motivo}",
                    flush=True)
            if t - ultima_amostra >= 0.2 or ceder:
                relatorio["amostras"].append({
                    "t_s": t, "intensidade": intensidade,
                    "pausa_feedback": pausa_feedback, "juntas": fotos,
                })
                ultima_amostra = t
            marco = int(t // 4)
            if marco != ultimo_print:
                ultimo_print = marco
                print(
                    f"t={t:.1f}s intensidade={intensidade:.2f} " + " ".join(
                        f"ID{f['id']}={f['atual_graus']:+.2f}°/{f['esforco_nm']:+.1f}Nm"
                        for f in fotos),
                    flush=True)
            if t >= total_s + 2.0:
                relatorio["status"] = "concluida"
                break
            proximo_tick += PERIODO_S
        relatorio["retorno_graus"] = {
            str(j.can_id): math.degrees(
                link.states[j.can_id]["position_rad"] - origens[j.can_id])
            for j in marcha.JOINTS if j.can_id in link.states}
        relatorio["intensidade_final"] = intensidade
    except KeyboardInterrupt:
        relatorio["status"] = "interrompida"
        relatorio["motivo"] = "parada solicitada"
        print("INTERROMPIDA: parada solicitada", flush=True)
    except Exception as exc:
        relatorio["status"] = "interrompida"
        relatorio["motivo"] = str(exc)
        print("INTERROMPIDA:", exc, flush=True)
    finally:
        if link.serial.is_open:
            relatorio["erros_desligamento"] = link.shutdown()
        else:
            relatorio["erros_desligamento"] = []
        relatorio["fd_enviado"] = True
        relatorio["torque_desligado_confirmado"] = False
        destino = pasta / "execucao.json"
        destino.write_text(json.dumps(relatorio, indent=2, ensure_ascii=False), encoding="utf-8")
        print("REGISTRO", destino, "STATUS", relatorio["status"], flush=True)
    return relatorio["status"] == "concluida"


def instalar_parada_por_sinal():
    """SIGINT e SIGTERM soltam os motores no finally, em vez de matar o processo."""

    def pedir_parada(signum, _frame):
        raise KeyboardInterrupt(signum)

    signal.signal(signal.SIGINT, pedir_parada)
    signal.signal(signal.SIGTERM, pedir_parada)


if __name__ == "__main__":
    instalar_parada_por_sinal()
    argumento = sys.argv[1] if len(sys.argv) > 1 else None
    ok = executar(argumento)
    sys.exit(0 if ok else 1)
