#!/usr/bin/env python3
# Teleop robot 4WD: mando Xbox360 (xpad, /dev/input/js0) -> Arduino Due (serial)
# Estilo carrera: RT acelera, LT retrocede, stick izq dirección. B = freno.
# D-pad arriba/abajo = sube/baja límite de velocidad. Watchdog en el Due (300 ms).
import os, struct, time, glob, sys

JS_DEV   = "/dev/input/js0"
EV_FMT   = "IhBB"; EV_SIZE = struct.calcsize(EV_FMT)
AX_LX, AX_LT, AX_RT, AX_DPX, AX_DPY = 0, 2, 5, 6, 7
BT_B = 1
DEADZONE = 0.12
MIN_PWM  = 28          # deadband de los motores
RATE_HZ  = 25

def find_serial():
    c = sorted(glob.glob("/dev/serial/by-id/*Arduino*")) or sorted(glob.glob("/dev/ttyACM*"))
    return c[0] if c else None

def open_serial():
    import serial
    port = find_serial()
    if not port: return None
    try:
        s = serial.Serial(port, 115200, timeout=0, write_timeout=0.2)
        print("serial:", port, flush=True); return s
    except Exception as e:
        print("serial err:", e, flush=True); return None

def open_js():
    try:
        fd = os.open(JS_DEV, os.O_RDONLY | os.O_NONBLOCK)
        print("mando conectado", flush=True); return fd
    except OSError:
        return None

def main():
    ax = {}; bt = {}
    limit = 0.5; last_dpy = 0
    js = None; ser = None; last_send = 0.0; last_l = last_r = None
    while True:
        if js is None:
            js = open_js()
            if js is None: time.sleep(1.0)
        if ser is None:
            ser = open_serial()
            if ser is None: time.sleep(1.0); continue
        # leer todos los eventos pendientes del mando
        if js is not None:
            try:
                while True:
                    data = os.read(js, EV_SIZE)
                    if len(data) < EV_SIZE: break
                    _t, val, typ, num = struct.unpack(EV_FMT, data)
                    typ &= ~0x80                      # aceptar eventos de init
                    if typ == 2: ax[num] = val
                    elif typ == 1: bt[num] = val
            except BlockingIOError:
                pass
            except OSError:
                print("mando desconectado", flush=True)
                try: os.close(js)
                except OSError: pass
                js = None; ax.clear(); bt.clear()
        # límite de velocidad con D-pad (flanco)
        dpy = ax.get(AX_DPY, 0)
        if dpy < -16000 and last_dpy >= -16000: limit = min(1.0, limit + 0.1); print("limite %d%%" % round(limit*100), flush=True)
        if dpy >  16000 and last_dpy <=  16000: limit = max(0.2, limit - 0.1); print("limite %d%%" % round(limit*100), flush=True)
        last_dpy = dpy
        # mezcla estilo carrera
        rt = (ax.get(AX_RT, -32767) + 32767) / 65534.0
        lt = (ax.get(AX_LT, -32767) + 32767) / 65534.0
        steer = ax.get(AX_LX, 0) / 32767.0
        if abs(steer) < DEADZONE: steer = 0.0
        else: steer = (abs(steer) - DEADZONE) / (1 - DEADZONE) * (1 if steer > 0 else -1)
        v = rt - lt                                   # -1..1
        if bt.get(BT_B): v = 0.0; steer = 0.0         # freno
        if abs(v) < 0.04: v = 0.0
        w = steer * 0.75                              # ganancia de giro
        if v == 0.0 and steer != 0.0: w = steer * 0.6 # pivote en el lugar
        l = max(-1.0, min(1.0, v + w)) * limit
        r = max(-1.0, min(1.0, v - w)) * limit
        def pwm(x):
            if abs(x) < 0.03: return 0
            s = 1 if x > 0 else -1
            return s * int(MIN_PWM + abs(x) * (255 - MIN_PWM))
        L, R = pwm(l), pwm(r)
        now = time.time()
        if now - last_send >= 1.0 / RATE_HZ:
            last_send = now
            if js is None: L = R = 0                  # sin mando => quieto
            try:
                ser.write(b"V %d %d\n" % (L, R))
                if (L, R) != (last_l, last_r) and (L, R) == (0, 0): ser.write(b"S\n")
                last_l, last_r = L, R
                # drenar telemetría para que no crezca el buffer
                ser.read(4096)
            except Exception as e:
                print("serial caido:", e, flush=True)
                try: ser.close()
                except Exception: pass
                ser = None
        time.sleep(0.005)

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt:
        pass
