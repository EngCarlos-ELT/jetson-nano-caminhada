#!/usr/bin/env python3
"""Mede o batente de cada motor pelo terminal, com o torque desligado.

Pede uma pose por vez. A pessoa gira o motor na mao e aperta Enter.
O unico quadro enviado e o FD, que pede o estado e deixa o motor solto.
O resultado sai em ensaios/batentes_*/medicao.json. No fim pergunta se estes
limites devem substituir a tabela de suporte_marcha.py. So grava se a
resposta for s. Motor sem os dois batentes permanece com o valor antigo.

Versao para Raspberry Pi 4. A CANable v2 entra em /dev/ttyACM0. Nao rode
junto com exo_master.py nem com a caminhada: os dois usam a mesma CANable.

  python3 medir_batentes.py
  python3 medir_batentes.py /dev/ttyACM0
  python3 medir_batentes.py --checar
"""

import json
import math
import re
import select
import sys
import time
from datetime import datetime
from pathlib import Path

import suporte_marcha as marcha


PORTA = "/dev/ttyACM0"
FD = bytes([255] * 7 + [253])
PRAZO_LEITURA_S = 0.5
# (id, nome, pose que a pessoa deve fazer, lado gravado)
re_lados = re.compile(r"ID([1-4]) frente ([+-]?\d+\.\d+), tras ([+-]?\d+\.\d+)")
re_tabela = re.compile(
    r"^\s*([1-4]): \(([+-]?\d+(?:\.\d+)?), ([+-]?\d+(?:\.\d+)?)\),",
    re.M)


PASSOS = (
    (1, "coxa esquerda", "perna para a frente", "frente"),
    (1, "coxa esquerda", "perna para tras", "tras"),
    (2, "joelho esquerdo", "estendido para a frente", "frente"),
    (2, "joelho esquerdo", "flexionado para tras", "tras"),
    (3, "coxa direita", "perna para a frente", "frente"),
    (3, "coxa direita", "perna para tras", "tras"),
    (4, "joelho direito", "estendido para a frente", "frente"),
    (4, "joelho direito", "flexionado para tras", "tras"),
)


def texto_do_passo(passo, indice, total):
    can_id, nome, pose, _lado = passo
    return "%d/%d  %s, ID %d, %s." % (indice, total, nome.capitalize(), can_id, pose)


def aceitar_leitura(estado, agora, prazo_s=PRAZO_LEITURA_S):
    """Graus se a resposta deste motor chegou agora. None se estiver velha."""
    if not estado:
        return None
    recebido = estado.get("received_at")
    if recebido is None or agora - recebido > prazo_s:
        return None
    graus = math.degrees(estado["position_rad"])
    if not math.isfinite(graus):
        return None
    return graus


def montar_resumo(registros):
    """Por motor, o angulo de cada lado e a faixa menor/maior quando os dois existem."""
    por_id = {}
    for item in registros:
        lados = por_id.setdefault(item["id"], {})
        lados[item["lado"]] = item["graus"]
    faixas = {}
    for can_id, lados in por_id.items():
        if "frente" in lados and "tras" in lados:
            faixas[can_id] = (min(lados["frente"], lados["tras"]),
                              max(lados["frente"], lados["tras"]))
    return por_id, faixas


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


def linha_ao_vivo(can_id, estado, agora):
    graus = aceitar_leitura(estado, agora)
    if graus is None:
        return "ID %d sem resposta. Espere o angulo e aperte Enter." % can_id
    return "ID %d  %+.2f°   esforco %+.2f Nm    Enter grava, q encerra" % (
        can_id, graus, estado["effort_nm"])


def esperar_tecla(link, can_id):
    """Atualiza o angulo ate o Enter. Devolve a leitura, ou None se a pessoa sair."""
    while True:
        pedir_estado(link)
        agora = time.monotonic()
        estado = link.states.get(can_id)
        sys.stdout.write("\r" + linha_ao_vivo(can_id, estado, agora).ljust(78))
        sys.stdout.flush()
        if sys.stdin.isatty():
            pronto, _gravar, _erro = select.select([sys.stdin], [], [], 0.1)
            if not pronto:
                continue
        sys.stdout.write("\n")
        sys.stdout.flush()
        linha = sys.stdin.readline()
        if linha.strip().lower() in ("q", "sair"):
            return None
        graus = aceitar_leitura(estado, time.monotonic())
        if graus is None:
            print("Sem leitura nova desse motor. Aperte Enter quando o angulo aparecer.",
                  flush=True)
            if not sys.stdin.isatty():
                return None
            continue
        return {
            "graus": graus,
            "esforco_nm": estado["effort_nm"],
            "temperatura_c": estado["temperature_c"],
            "falha": estado["fault"],
        }


def medir(link):
    registros = []
    total = len(PASSOS)
    print(
        "Torque desligado. Apoie o exo e gire o motor pedido ate o batente.\n"
        "Enter grava o angulo dessa pose. q encerra e guarda o que ja foi gravado.\n",
        flush=True)
    for indice, passo in enumerate(PASSOS, start=1):
        can_id, nome, pose, lado = passo
        print(texto_do_passo(passo, indice, total), flush=True)
        print("Gire: %s, %s." % (nome, pose), flush=True)
        leitura = esperar_tecla(link, can_id)
        if leitura is None:
            print("Medicao encerrada antes do fim.", flush=True)
            break
        item = {
            "id": can_id,
            "nome": nome,
            "pose": pose,
            "lado": lado,
            "graus": leitura["graus"],
            "esforco_nm": leitura["esforco_nm"],
            "temperatura_c": leitura["temperatura_c"],
            "falha": leitura["falha"],
        }
        registros.append(item)
        print("Gravado: %s, ID %d, %s = %+.2f°" % (nome, can_id, pose, leitura["graus"]),
              flush=True)
        print(flush=True)
    return registros


def quer_atualizar(resposta):
    """True para s, False para n, None se a resposta nao serve."""
    texto = resposta.strip().lower()
    if texto in ("s", "sim"):
        return True
    if texto in ("n", "nao", "não"):
        return False
    return None


def bloco_de_limites(texto):
    """Trecho do comentario e da tabela, ou None se o arquivo mudou de forma."""
    inicio = texto.find("# Tupla (menor, maior) em graus MIT.")
    marca = texto.find("MEASURED_BATENT_LIMITS_DEG = {", inicio if inicio >= 0 else 0)
    fim = texto.find("\n}\n", marca if marca >= 0 else 0)
    if inicio < 0 or marca < 0 or fim < 0 or not inicio < marca < fim:
        return None
    return inicio, fim + 3


def lados_do_comentario(texto):
    achados = {}
    for can_id, frente, tras in re_lados.findall(texto):
        achados[int(can_id)] = (float(frente), float(tras))
    return achados


def texto_do_bloco(lados, faixas, data):
    def comentario(can_id):
        frente, tras = lados[can_id]
        return "ID%d frente %+.2f, tras %+.2f" % (can_id, frente, tras)

    linhas = [
        "# Tupla (menor, maior) em graus MIT. Medido em %s:" % data,
        "# %s. %s." % (comentario(1), comentario(2)),
        "# %s. %s." % (comentario(3), comentario(4)),
        "# No joelho, frente e a extensao: ID2 no menor, ID4 no maior.",
        "MEASURED_BATENT_LIMITS_DEG = {",
    ]
    for can_id in (1, 2, 3, 4):
        baixo, alto = faixas[can_id]
        linhas.append("    %d: (%.2f, %.2f)," % (can_id, baixo, alto))
    linhas.append("}")
    return "\n".join(linhas) + "\n"


def aplicar_limites_no_texto(texto, lados_novos, faixas_novas, data):
    """Devolve o arquivo com os motores medidos. Os outros ficam como estavam."""
    bloco = bloco_de_limites(texto)
    if bloco is None:
        raise ValueError("Bloco de limites nao encontrado em suporte_marcha.py")
    inicio, fim = bloco
    lados = lados_do_comentario(texto)
    if sorted(lados) != [1, 2, 3, 4]:
        raise ValueError("O comentario nao tem frente e tras dos quatro motores")
    faixas = {}
    for can_id, baixo, alto in re_tabela.findall(texto[inicio:fim]):
        faixas[int(can_id)] = (float(baixo), float(alto))
    if sorted(faixas) != [1, 2, 3, 4]:
        raise ValueError("A tabela nao tem os quatro motores")
    for can_id, (baixo, alto) in faixas_novas.items():
        if not (math.isfinite(baixo) and math.isfinite(alto) and baixo < alto):
            raise ValueError("Faixa invalida no ID %d" % can_id)
        faixas[can_id] = (baixo, alto)
        if can_id not in lados_novos or "frente" not in lados_novos[can_id] or "tras" not in lados_novos[can_id]:
            raise ValueError("ID %d sem os dois lados" % can_id)
        lados[can_id] = (lados_novos[can_id]["frente"], lados_novos[can_id]["tras"])
    novo = texto_do_bloco(lados, faixas, data)
    return texto[:inicio] + novo + texto[fim:]


def atualizar_suporte(lados, faixas):
    """Grava a tabela do Jetson. Nao mexe nas copias de Raspberry_v4 e Windows."""
    caminho = Path(__file__).resolve().parent / "suporte_marcha.py"
    texto = caminho.read_text(encoding="utf-8")
    data = datetime.now().strftime("%Y-%m-%d")
    novo = aplicar_limites_no_texto(texto, lados, faixas, data)
    caminho.write_text(novo, encoding="utf-8")
    return caminho


def perguntar_atualizacao(faixas):
    if not faixas:
        print(
            "Nenhum motor tem os dois batentes. suporte_marcha.py nao foi alterado.",
            flush=True)
        return False
    print("Estes limites podem entrar em suporte_marcha.py:", flush=True)
    for can_id in sorted(faixas):
        baixo, alto = faixas[can_id]
        print("  ID %d  %+.2f° a %+.2f°" % (can_id, baixo, alto), flush=True)
    faltantes = [can_id for can_id in (1, 2, 3, 4) if can_id not in faixas]
    if faltantes:
        print(
            "Sem os dois batentes, ficam como estao: ID %s."
            % ", ".join(str(can_id) for can_id in faltantes),
            flush=True)
    while True:
        sys.stdout.write("Atualizar suporte_marcha.py com estes limites? s/n ")
        sys.stdout.flush()
        resposta = sys.stdin.readline()
        if resposta == "":
            print("suporte_marcha.py nao foi alterado.", flush=True)
            return False
        escolha = quer_atualizar(resposta)
        if escolha is None:
            print("Responda s ou n.", flush=True)
            continue
        return escolha


def mostrar_resumo(registros):
    lados, faixas = montar_resumo(registros)
    print("Resumo.", flush=True)
    vistos = []
    for can_id, nome, _pose, _lado in PASSOS:
        if can_id in vistos:
            continue
        vistos.append(can_id)
        baixo_gravado, alto_gravado = marcha.MEASURED_BATENT_LIMITS_DEG[can_id]
        print("ID %d  %s" % (can_id, nome), flush=True)
        for item in registros:
            if item["id"] != can_id:
                continue
            print("  %s  %+.2f°" % (item["pose"], item["graus"]), flush=True)
        if can_id in faixas:
            baixo, alto = faixas[can_id]
            print("  faixa medida  %+.2f° a %+.2f°" % (baixo, alto), flush=True)
            if abs(alto - baixo) < 15.0:
                print("  AVISO: os dois batentes ficaram proximos.", flush=True)
        else:
            print("  faixa medida  incompleta", flush=True)
        print("  gravado hoje na marcha  %+.2f° a %+.2f°" % (baixo_gravado, alto_gravado),
              flush=True)
    return lados, faixas


def salvar(porta, registros, faixas, alterado):
    if not registros:
        print("Nada gravado.", flush=True)
        return None
    pasta = Path(__file__).resolve().parent / "ensaios" / (
        "batentes_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    pasta.mkdir(parents=True, exist_ok=True)
    destino = pasta / "medicao.json"
    corpo = {
        "porta": porta,
        "medicoes": registros,
        "faixas_graus": {str(can_id): [baixo, alto] for can_id, (baixo, alto) in faixas.items()},
        "gravado_na_marcha_graus": {
            str(can_id): list(par) for can_id, par in marcha.MEASURED_BATENT_LIMITS_DEG.items()},
        "limites_da_marcha_alterados": alterado,
        "ids_atualizados": sorted(faixas) if alterado else [],
    }
    destino.write_text(json.dumps(corpo, indent=2, ensure_ascii=False), encoding="utf-8")
    print("REGISTRO", destino, flush=True)
    return destino


def checar():
    texto = texto_do_passo(PASSOS[6], 7, len(PASSOS))
    if "Joelho direito" not in texto or "ID 4" not in texto or "estendido para a frente" not in texto:
        raise SystemExit("O pedido do joelho direito nao descreve a pose: %s" % texto)
    ids = [passo[0] for passo in PASSOS]
    if sorted(ids) != [1, 1, 2, 2, 3, 3, 4, 4]:
        raise SystemExit("Cada motor precisa dos dois batentes")
    if len({(passo[0], passo[3]) for passo in PASSOS}) != 8:
        raise SystemExit("Ha pose repetida")
    agora = 10.0
    if aceitar_leitura(None, agora) is not None:
        raise SystemExit("Leitura vazia foi aceita")
    if aceitar_leitura({"position_rad": 0.0, "received_at": agora - 1.0}, agora) is not None:
        raise SystemExit("Leitura velha foi aceita")
    if aceitar_leitura({"position_rad": float("nan"), "received_at": agora}, agora) is not None:
        raise SystemExit("Leitura invalida foi aceita")
    graus = aceitar_leitura({"position_rad": math.radians(12.5), "received_at": agora}, agora)
    if graus is None or abs(graus - 12.5) > 1e-9:
        raise SystemExit("A leitura fresca nao virou graus")
    registros = [
        {"id": 2, "lado": "frente", "graus": 1.04},
        {"id": 2, "lado": "tras", "graus": 92.60},
        {"id": 3, "lado": "frente", "graus": -86.70},
        {"id": 3, "lado": "tras", "graus": 29.00},
        {"id": 4, "lado": "frente", "graus": 9.34},
    ]
    _lados, faixas = montar_resumo(registros)
    if abs(faixas[2][0] - 1.04) > 1e-9 or abs(faixas[2][1] - 92.60) > 1e-9:
        raise SystemExit("A faixa do joelho esquerdo nao usa os dois batentes")
    if abs(faixas[3][0] - (-86.70)) > 1e-9 or abs(faixas[3][1] - 29.00) > 1e-9:
        raise SystemExit("A faixa da coxa direita nao usa o menor e o maior")
    if 4 in faixas:
        raise SystemExit("Um batente so nao pode fechar a faixa")
    if quer_atualizar("s") is not True or quer_atualizar("sim") is not True:
        raise SystemExit("A resposta sim nao atualiza")
    if quer_atualizar("n") is not False or quer_atualizar("não") is not False:
        raise SystemExit("A resposta nao ainda atualizaria")
    if quer_atualizar("") is not None or quer_atualizar("talvez") is not None:
        raise SystemExit("Resposta incerta foi aceita")
    amostra = (
        "# Tupla (menor, maior) em graus MIT. Medido em 2026-11-14:\n"
        "# ID1 frente +97.03, tras -18.48. ID2 frente +1.04, tras +92.60.\n"
        "# ID3 frente -86.70, tras +29.00. ID4 frente +9.34, tras -83.79.\n"
        "# No joelho, frente e a extensao: ID2 no menor, ID4 no maior.\n"
        "MEASURED_BATENT_LIMITS_DEG = {\n"
        "    1: (-18.48, 97.03),\n"
        "    2: (1.04, 92.60),\n"
        "    3: (-86.70, 29.00),\n"
        "    4: (-83.79, 9.34),\n"
        "}\n"
        "\n"
        "def smooth_c3(u):\n"
        "    return u\n"
    )
    novo = aplicar_limites_no_texto(
        amostra,
        {2: {"frente": 2.5, "tras": 90.0}},
        {2: (2.5, 90.0)},
        "2026-11-17")
    if "    2: (2.50, 90.00)," not in novo or "    1: (-18.48, 97.03)," not in novo:
        raise SystemExit("A tabela nao trocou so o motor medido")
    if "ID2 frente +2.50, tras +90.00" not in novo or "ID1 frente +97.03, tras -18.48" not in novo:
        raise SystemExit("O comentario nao preservou os motores de fora")
    if "def smooth_c3(u):" not in novo:
        raise SystemExit("O resto do arquivo foi cortado")
    try:
        aplicar_limites_no_texto(amostra, {2: {"frente": 1.0}}, {2: (1.0, 2.0)}, "2026-11-17")
    except ValueError:
        pass
    else:
        raise SystemExit("Faixa sem os dois lados foi aceita")
    print("Logica ok. 8 poses. A tabela so muda se a resposta for s. CAN nao foi aberta.",
          flush=True)


def main():
    if "--checar" in sys.argv:
        checar()
        return
    porta = sys.argv[1] if len(sys.argv) > 1 else PORTA
    link = abrir(porta)
    registros = []
    try:
        pedir_estado(link)
        registros = medir(link)
    except KeyboardInterrupt:
        print("\nMedicao interrompida.", flush=True)
    finally:
        try:
            link.shutdown()
        except Exception as exc:
            print("AVISO ao soltar: %s" % exc, flush=True)
    if registros:
        lados, faixas = mostrar_resumo(registros)
        alterar = perguntar_atualizacao(faixas)
        if alterar:
            try:
                caminho = atualizar_suporte(lados, faixas)
            except Exception as exc:
                alterar = False
                print("Nao atualizei suporte_marcha.py: %s" % exc, flush=True)
            else:
                print("Atualizado: %s" % caminho, flush=True)
                print("IDs gravados: %s." % ", ".join(str(can_id) for can_id in sorted(faixas)),
                      flush=True)
        elif faixas:
            print("suporte_marcha.py ficou como estava.", flush=True)
        salvar(porta, registros, faixas, alterar)


if __name__ == "__main__":
    main()
