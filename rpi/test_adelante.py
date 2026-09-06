import sys, time, serial
# Prueba del firmware teleop: lee el banner "R listo", manda V <pwm> <pwm> durante N s y
# muestra el avance de los 4 encoders (fl rl fr rr). Todos deben ser POSITIVOS y parecidos.
pwm=int(sys.argv[1]) if len(sys.argv)>1 else 70; segs=float(sys.argv[2]) if len(sys.argv)>2 else 2.0
s=serial.Serial("/dev/ttyACM0",115200,timeout=0.05); buf=b""; T=[]; R=[]
def pump():
    global buf
    buf+=s.read(4096)
    while b"\n" in buf:
        ln,buf=buf.split(b"\n",1); ln=ln.strip().decode("utf-8","replace")
        if ln.startswith("T "): T.append(ln)
        elif ln.startswith("R "): R.append(ln)
t0=time.time()
while time.time()-t0<7 and not R: pump(); time.sleep(0.02)
print("banner:", R[-1] if R else "(no llego R listo)")
while time.time()-t0<8.5: pump(); time.sleep(0.02)
t_ini=T[-1] if T else None
t1=time.time()
while time.time()-t1<segs:
    s.write(("V %d %d\n"%(pwm,pwm)).encode()); pump(); time.sleep(0.1)
s.write(b"S\n"); t2=time.time()
while time.time()-t2<0.8: pump(); time.sleep(0.02)
print("inicio:", t_ini); print("final: ", T[-1] if T else None)
if t_ini and T:
    a=[int(x) for x in t_ini.split()[1:5]]; b=[int(x) for x in T[-1].split()[1:5]]
    d=[y-x for x,y in zip(a,b)]
    print("avance fl rl fr rr =", d, " IMU:", "SI" if len(T[-1].split())==7 else "NO")
    print("VEREDICTO:", "OK todos positivos" if all(v>0 for v in d) else "MAL: hay alguno <= 0")
