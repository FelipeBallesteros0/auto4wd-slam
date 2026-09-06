import sys, time, serial
# uso: serial_cmd.py <chars> <segundos_lectura> [espera_banner=5] [pausa_entre_chars=0]
cmd=sys.argv[1]; secs=float(sys.argv[2])
espera=float(sys.argv[3]) if len(sys.argv)>3 else 5.0
gap=float(sys.argv[4]) if len(sys.argv)>4 else 0.0
s=serial.Serial("/dev/ttyACM0",115200,timeout=0.2)
def lee(hasta):
    while time.time()<hasta:
        d=s.read(4096)
        if d: sys.stdout.write(d.decode("utf-8","replace")); sys.stdout.flush()
lee(time.time()+espera)
for c in cmd:
    s.write(c.encode()); s.flush()
    if gap: lee(time.time()+gap)
lee(time.time()+secs)
