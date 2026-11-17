"""Faixas operacionais: joelhos podem apoiar no batente medido de extensão."""
import math


def operational_limits_deg(can_id, low, high, margin=1.0):
    if not all(math.isfinite(v) for v in (low, high, margin)) or margin < 0 or low >= high:
        raise ValueError('Batentes ou margem inválidos')
    # ID2 flexiona em MIT+; ID4 flexiona em MIT-. Não ampliar batentes.
    safe_low = low if can_id == 2 else low + margin
    safe_high = high if can_id == 4 else high - margin
    if safe_low >= safe_high:
        raise ValueError('Faixa operacional vazia')
    return safe_low, safe_high


def extension_safe_feedforward(can_id, actual_deg, low, high, ff_nm):
    """Não acumular pressão FF para fora do curso ao tocar extensão."""
    if ((can_id == 2 and actual_deg <= low + .25 and ff_nm < 0) or
            (can_id == 4 and actual_deg >= high - .25 and ff_nm > 0)):
        return 0.0
    return ff_nm


def inward_quantized_target(can_id, position_rad, low, high):
    """Reserve one position quantum inward so encoder rounding cannot cross a stop."""
    quantum = 25.0 / 65535
    if can_id == 2:
        return max(math.radians(low)+quantum, position_rad)
    if can_id == 4:
        return min(math.radians(high)-quantum, position_rad)
    return position_rad
