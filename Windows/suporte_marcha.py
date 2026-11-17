"""Enlace CAN e curva de passos usados pela caminhada.

Trecho necessario do controlador de bancada, sem a interface Windows.
Compativel com Python 3.6 do Ubuntu 18 no Jetson Nano.
"""
import bisect
import csv
import json
import math
import time
from collections import deque
from pathlib import Path


class Joint(object):
    def __init__(self, name, can_id, model, sign, start, velocity_scale,
                 effort_scale, budget, ff_limit, trip):
        self.name = name
        self.can_id = can_id
        self.model = model
        self.sign = sign
        self.start = start
        self.velocity_scale = velocity_scale
        self.effort_scale = effort_scale
        self.budget = budget
        self.ff_limit = ff_limit
        self.trip = trip


JOINTS = (
    Joint('hip_left', 1, 'AK80-64', -1, 0, 8, 144, 6, 2.5, 7),
    Joint('knee_left', 2, 'AK80-8-KV60', 1, 0, 37.5, 32, 4, 1.2, 5),
    Joint('hip_right', 3, 'AK80-64', 1, 24, 8, 144, 6, 2.5, 7),
    Joint('knee_right', 4, 'AK80-8-KV60', -1, 24, 37.5, 32, 4, 1.2, 5),
)
KP, KP_MAX, KD = 60.0, 500.0, 4.5
KNEE_SWING_FLEXION_HOLD_FRACTION = .70
STARTUP_FEEDBACK_TIMEOUT_S = 0.25
STARTUP_FEEDBACK_ATTEMPTS = 3
ENCODER_MIN_WINDOW_S = .03
ENCODER_MIN_POINTS = 3
ENCODER_FAST_PAIR_THRESHOLD_RAD_S = 0.5
RAW_SPEED_HARD_TRIP_RAD_S = 3.0
WATCHDOG = 0.12
MAX_FIGURE_EXCURSION_DEG = 60.0
FIGURE_REFERENCE_CSV = Path(__file__).with_name('referencia_figura_ciclo_marcha.csv')
FIGURE_METADATA_JSON = FIGURE_REFERENCE_CSV.with_suffix('.json')
# Tupla (menor, maior) em graus MIT. Medido em 2026-11-17:
# ID1 frente +92.49, tras -24.43. ID2 frente +0.14, tras +92.60.
# ID3 frente -81.14, tras +34.72. ID4 frente +9.52, tras -83.94.
# No joelho, frente e a extensao: ID2 no menor, ID4 no maior.
MEASURED_BATENT_LIMITS_DEG = {
    1: (-24.43, 92.49),
    2: (0.14, 92.60),
    3: (-81.14, 34.72),
    4: (-83.94, 9.52),
}


def smooth_c3(u):
    u = max(0.0, min(1.0, u))
    return u**4 * (35 + u * (-84 + u * (70 - 20*u)))


def direction_sign(joint, anatomical=False):
    if anatomical and joint.name in ('hip_left', 'hip_right'):
        return -joint.sign
    return joint.sign


def load_figure_profile(cycle_duration=8.0, max_excursion_deg=None):
    if not math.isfinite(cycle_duration) or cycle_duration < 1:
        raise ValueError('A duracao do ciclo da figura deve ser >=1 s')
    if max_excursion_deg is not None and (
            not math.isfinite(max_excursion_deg) or not 0 < max_excursion_deg <= MAX_FIGURE_EXCURSION_DEG):
        raise ValueError('A excursao maxima da figura deve estar entre 0 e %g graus' % MAX_FIGURE_EXCURSION_DEG)
    rows = list(csv.DictReader(FIGURE_REFERENCE_CSV.read_text(encoding='utf-8').splitlines()))
    if len(rows) != 101 or not {'phase_percent', 'hip_flexion_deg', 'knee_flexion_deg'}.issubset(rows[0]):
        raise ValueError('CSV da figura deve conter 101 pontos de quadril e joelho')
    phases, profile = [], []
    for row in rows:
        phase = float(row['phase_percent'])
        values = [float(row['hip_flexion_deg']), float(row['knee_flexion_deg'])]
        if not math.isfinite(phase) or not all(math.isfinite(v) for v in values):
            raise ValueError('CSV da figura contem valor nao finito')
        if phases and phase <= phases[-1]:
            raise ValueError('Fases do grafico devem ser estritamente crescentes')
        phases.append(phase*cycle_duration/100)
        profile.append(values)
    if phases[0] != 0 or not math.isclose(phases[-1], cycle_duration, abs_tol=1e-8):
        raise ValueError('CSV da figura deve cobrir de 0 a 100 por cento do ciclo')
    profile[-1] = list(profile[0])
    metadata = json.loads(FIGURE_METADATA_JSON.read_text(encoding='utf-8'))
    if metadata.get('source_kind') != 'user_supplied_plot_digitization':
        raise ValueError('A figura nao corresponde ao perfil digitizado fornecido pelo usuario')
    metadata['cycle_duration_s'] = cycle_duration
    metadata['preview_sample_rate_hz'] = 50.0
    metadata['left_right_phase_offset_percent'] = 50
    metadata['phase_assignment'] = (
        'left leg starts at source phase50 percent (forward first); '
        'right leg starts at phase0 percent; then alternate every half-cycle')
    metadata['scale_factor'] = 1.0
    if max_excursion_deg is not None:
        full_peak = max(abs(figure_reference_deg(joint, sample_time, 'ambas',
                              (phases, profile, metadata)))
                        for joint in JOINTS for sample_time in phases)
        if full_peak <= 1e-9:
            raise ValueError('A figura nao apresenta excursao para aplicar escala')
        metadata['unscaled_peak_excursion_deg'] = full_peak
        metadata['requested_max_excursion_deg'] = max_excursion_deg
        metadata['scale_factor'] = max_excursion_deg/full_peak
    return phases, profile, metadata


def figure_reference_deg(joint, t, leg, profile_data):
    times, profile, _metadata = profile_data
    is_left = joint.name.endswith('left')
    if leg != 'ambas' and is_left != (leg == 'esquerda'):
        return 0.0
    duration = times[-1]
    phase = (t+duration/2) % duration if is_left else t
    anchor_phase = duration/2 if is_left else 0.0
    column = 0 if joint.name.startswith('hip_') else 1

    def angle_at(sample_time):
        index = min(len(times)-2, max(0, bisect.bisect_right(times, sample_time)-1))
        blend = (sample_time-times[index])/(times[index+1]-times[index])
        return profile[index][column] + blend*(profile[index+1][column]-profile[index][column])

    offset = angle_at(phase)-angle_at(anchor_phase)
    return offset*direction_sign(joint, anatomical=True)*_metadata.get('scale_factor', 1.0)


def figure_steps_reference_deg(joint, t, step_count, leg, profile_data):
    times, _profile, metadata = profile_data
    if leg != 'ambas':
        raise ValueError('Passos alternados exigem as duas pernas')
    cycle_duration = times[-1]
    walk_duration = cycle_duration*step_count/2
    if t <= 0 or t >= walk_duration:
        return 0.0
    scale = metadata.get('requested_max_excursion_deg', 28.0)/28.0
    ramp_duration = cycle_duration/4
    envelope = min(smooth_c3(t/ramp_duration),
                   smooth_c3((walk_duration-t)/ramp_duration))
    phase = 2*math.pi*t/cycle_duration
    phase_sine = math.sin(phase)
    end_envelope = smooth_c3((walk_duration-t)/ramp_duration)
    if joint.name.startswith('hip_'):
        hip_peak = metadata.get('requested_hip_peak_deg', 18.0*scale)
        return hip_peak*envelope*phase_sine

    knee_peak = metadata.get('knee_flexion_deg', 40.0)
    preflex_duration = cycle_duration/4
    swing_duration = cycle_duration/2

    def flexion_level(rear_phase):
        since_rear = (t-rear_phase*cycle_duration) % cycle_duration
        if since_rear <= swing_duration:
            hold = KNEE_SWING_FLEXION_HOLD_FRACTION*swing_duration
            return 1.0-smooth_c3((since_rear-hold)/(swing_duration-hold))
        preflex_start = cycle_duration-preflex_duration
        if since_rear >= preflex_start:
            return smooth_c3((since_rear-preflex_start)/preflex_duration)
        return 0.0

    if joint.name == 'knee_left':
        rear_phase = .75
        if t < cycle_duration/4:
            rise_duration = cycle_duration/8
            kickoff = (knee_peak*smooth_c3(t/rise_duration)*
                       (1.0-smooth_c3((t-rise_duration)/rise_duration)))
            return kickoff
        return knee_peak*envelope*flexion_level(rear_phase)

    if joint.name == 'knee_right':
        rear_phase = .25
        return -knee_peak*end_envelope*flexion_level(rear_phase)
    raise ValueError('Junta sem referencia de passos: %s' % joint.name)


def encode_command(position, joint, ff=0.0, kp=KP, velocity=0.0, kd=KD):
    if not all(math.isfinite(x) for x in (position, ff, kp, velocity, kd)):
        raise ValueError('Comando nao finito')
    if not (-12.5 <= position <= 12.5 and 0 <= kp <= KP_MAX and 0 <= kd <= 5 and
            0 <= joint.ff_limit <= 6 and abs(ff) <= joint.ff_limit and
            abs(velocity) <= joint.velocity_scale and joint.effort_scale in (32, 144)):
        raise ValueError('Comando MIT fora dos limites do perfil')

    def encode(value, low, high, bits):
        return max(0, min((1<<bits)-1, int((value-low)*(1<<bits)/(high-low))))

    p = encode(position, -12.5, 12.5, 16)
    v = encode(velocity, -joint.velocity_scale, joint.velocity_scale, 12)
    k = encode(kp, 0, 500, 12)
    d = encode(kd, 0, 5, 12)
    torque = encode(ff, -joint.effort_scale, joint.effort_scale, 12)
    return bytes([p>>8, p&255, v>>4, ((v&15)<<4)|(k>>8), k&255,
                  d>>4, ((d&15)<<4)|(torque>>8), torque&255])


def frame(can_id, data):
    return ('t%03X%X%s\r' % (can_id, len(data), data.hex().upper())).encode('ascii')


def decode_reply(line):
    if not line.startswith(b't') or len(line) != 21:
        return None
    try:
        arb = int(line[1:4], 16)
        if int(line[4:5], 16) != 8:
            return None
        payload = bytes.fromhex(line[5:].decode('ascii'))
    except (ValueError, UnicodeError):
        return None
    joint = next((j for j in JOINTS if j.can_id == payload[0]), None)
    if joint is None or arb not in (0, joint.can_id):
        return None
    return joint.can_id, {
        'position_rad': int.from_bytes(payload[1:3], 'big')*25/65535 - 12.5,
        'velocity_rad_s': ((payload[3]<<4)|(payload[4]>>4))*2*joint.velocity_scale/4095-joint.velocity_scale,
        'effort_nm': (((payload[4]&15)<<8)|payload[5])*2*joint.effort_scale/4095-joint.effort_scale,
        'temperature_c': payload[6]-40, 'temperature_raw': payload[6], 'fault': payload[7],
        'raw': payload.hex(), 'reply_can_id': arb,
    }


class EncoderMonitor(object):
    def __init__(self):
        self.points = deque()

    def update(self, now, position):
        if self.points and now-self.points[-1][0] < .015:
            return self.value(now)
        self.points.append((now, position))
        while len(self.points) > 1 and now-self.points[0][0] > .16:
            self.points.popleft()
        return self.value(now)

    def value(self, now):
        if (len(self.points) < ENCODER_MIN_POINTS or
                self.points[-1][0]-self.points[0][0] < ENCODER_MIN_WINDOW_S):
            return None
        if now-self.points[-1][0] > WATCHDOG:
            return None
        points = list(self.points)
        fast = max(abs((b[1]-a[1])/(b[0]-a[0])) for a, b in zip(points, points[1:]))
        if fast > ENCODER_FAST_PAIR_THRESHOLD_RAD_S:
            return fast
        center = points[0][0]
        ts = [p[0]-center for p in points]
        mean_t = sum(ts)/len(ts)
        mean_p = sum(p[1] for p in points)/len(points)
        slope = sum((t-mean_t)*(p[1]-mean_p) for t, p in zip(ts, points)) / sum((t-mean_t)**2 for t in ts)
        return abs(slope)


class Link(object):
    def __init__(self, port):
        import serial
        self.serial = serial.Serial(port=None, baudrate=115200, timeout=0,
                                    write_timeout=0.1)
        self.serial.port = port
        self.serial.dtr = self.serial.rts = False
        self.buffer = bytearray()
        self.states = {}
        self.tx = 0
        self.rx = {j.can_id: 0 for j in JOINTS}
        self.monitors = {j.can_id: EncoderMonitor() for j in JOINTS}
        self.cancel_file = None

    def write(self, data):
        if self.serial.write(data) != len(data):
            raise RuntimeError('Escrita serial incompleta')
        self.tx += data.count(b't')

    def poll(self):
        if self.cancel_file is not None and self.cancel_file.exists():
            raise RuntimeError('Parada solicitada pela interface; encerrando movimento')
        self.buffer.extend(self.serial.read(self.serial.in_waiting))
        if b'\x07' in self.buffer:
            raise RuntimeError('CANable rejeitou comando SLCAN')
        if len(self.buffer) > 16384:
            raise RuntimeError('Buffer serial sem enquadramento valido')
        while b'\r' in self.buffer:
            line, _, rest = self.buffer.partition(b'\r')
            self.buffer[:] = rest
            parsed = decode_reply(line)
            if parsed:
                can_id, state = parsed
                state['received_at'] = time.monotonic()
                state['encoder_peak_speed_rad_s'] = self.monitors[can_id].update(
                    state['received_at'], state['position_rad'])
                self.states[can_id] = state
                self.rx[can_id] += 1

    def shutdown(self, can_ids=None):
        errors = []
        if self.serial.is_open:
            ids = [j.can_id for j in JOINTS if can_ids is None or j.can_id in can_ids]
            for _ in range(3):
                for can_id in ids:
                    try:
                        self.write(frame(can_id, bytes([255]*7+[253])))
                    except Exception as exc:
                        errors.append(str(exc))
                time.sleep(.02)
            try:
                self.write(b'C\r')
            except Exception as exc:
                errors.append(str(exc))
            self.serial.close()
        return errors


def request_fresh_feedback(link, joints, can_ids, send_request, phase='startup', trace=None):
    selected = set(can_ids)
    attempts = 0
    pending = set(selected)
    while pending and attempts < STARTUP_FEEDBACK_ATTEMPTS:
        attempts += 1
        attempt_started = time.monotonic()
        baselines = {can_id: link.rx[can_id] for can_id in selected}
        for joint in joints:
            if joint.can_id in selected:
                send_request(joint)
        deadline = time.monotonic() + STARTUP_FEEDBACK_TIMEOUT_S
        pending = set(selected)
        while pending and time.monotonic() < deadline:
            link.poll()
            pending = {can_id for can_id in selected
                       if link.rx[can_id] <= baselines[can_id]}
            if pending:
                time.sleep(.001)
        if trace is not None:
            now = time.monotonic()
            trace.append({
                'phase': phase,
                'attempt': attempts,
                'missing_ids': sorted(pending),
                'elapsed_s': now-attempt_started,
            })
    return pending, attempts
