#!/usr/bin/env python3
# encoding: utf-8
"""
Caracterizacion de motores + encoders del JetAuto (placa I2C 0x34, bus 1).
Corre DENTRO del contenedor del Nano, con el chassis detenido (uso exclusivo del I2C).
Robot ELEVADO (ruedas al aire). Maneja a baja velocidad y lee encoders (reg 60) para
determinar: sentido por motor, mapeo motor_id<->canal-encoder, pulsos/s, simetria, y
el patron mecanum para vx+/vy+/wz+.
"""
import smbus2, struct, time, math

ADDR = 0x34
REG_TYPE = 20
REG_SPEED = 50          # speed reg = 50 + motor_id (1..4), int8 [-100,100]
REG_ENC = 60            # 16 bytes = 4 x int32 LE (pulsos acumulados)

bus = smbus2.SMBus(1)

def set_type():
    bus.write_i2c_block_data(ADDR, REG_TYPE, [3])   # JGB37

def set_id(mid, sp):
    bus.write_i2c_block_data(ADDR, REG_SPEED + mid, [int(sp) & 0xFF])

def stop():
    for mid in (1, 2, 3, 4):
        set_id(mid, 0)

def read_enc():
    return list(struct.unpack('<iiii', bytes(bus.read_i2c_block_data(ADDR, REG_ENC, 16))))

SP = 35     # velocidad de prueba (de -100..100)
T = 1.5     # s por prueba

set_type(); stop(); time.sleep(0.6)

print("=== BARRIDO POR motor_id (1..4) ===")
chan_for_id = {}
for mid in (1, 2, 3, 4):
    stop(); time.sleep(0.4); e0 = read_enc()
    set_id(mid, SP); time.sleep(T); e1 = read_enc(); stop(); time.sleep(0.4)
    d = [e1[i] - e0[i] for i in range(4)]
    amax = max(range(4), key=lambda i: abs(d[i]))
    chan_for_id[mid] = amax
    sign = '+' if d[amax] > 0 else '-'
    print("motor_id %d @ sp+%d: Denc=%s -> canal %d, signo %s, pulsos/s %.0f"
          % (mid, SP, d, amax, sign, d[amax] / T))

# ---- Mecanum: replica MecanumChassis.set_velocity + motor_board.set_speeds ----
A, B, D, PPC = 103.0, 97.0, 96.5, 4320.0
IDX2ID = {0: 3, 1: 4, 2: 1, 3: 2}

def s2p(mm):
    return mm / (math.pi * D) * PPC * 0.01

def clamp8(v):
    v = int(v)
    return 100 if v > 100 else (-100 if v < -100 else v)

def mecanum(vx, vy, wz):
    lx, ly = vx * 1000.0, vy * 1000.0
    speed = math.sqrt(lx * lx + ly * ly)
    direction = math.atan2(ly, lx)
    direction = (2 * math.pi + direction) if direction < 0 else direction
    vxx = speed * math.sin(direction)
    vyy = speed * math.cos(direction)
    vp = wz * (A + B)
    v1 = vyy - vxx - vp
    v2 = vyy + vxx + vp
    v3 = vyy - vxx + vp
    v4 = vyy + vxx - vp
    vs = [int(s2p(v)) for v in [-v2, v3, v1, -v4]]
    for idx, sp in enumerate(vs):
        set_id(IDX2ID[idx], clamp8(sp))
    return [clamp8(x) for x in vs]

print("\n=== MECANUM (vx+, vy+, wz+) — Denc por canal ===")
for name, (vx, vy, wz) in [("vx+ adelante", (0.08, 0, 0)),
                           ("vy+ izquierda", (0, 0.08, 0)),
                           ("wz+ CCW", (0, 0, 0.6))]:
    stop(); time.sleep(0.5); e0 = read_enc()
    vs = mecanum(vx, vy, wz); time.sleep(T); e1 = read_enc(); stop(); time.sleep(0.5)
    d = [e1[i] - e0[i] for i in range(4)]
    print("%-14s speeds=%s  Denc=%s" % (name, vs, d))

stop()
print("\nchan_for_id (motor_id->canal_encoder):", chan_for_id)
print("=== FIN ===")
