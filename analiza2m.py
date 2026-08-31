#!/usr/bin/env python3
# Analiza el log del test de 2 m: uso
#   ssh felipe@<pi> cat /tmp/test2m.log | python3 analiza2m.py [--real 2.00]
# Toma la primera y última línea "estable" (robot quieto) y compara:
# distancia por encoders vs por láser (pared) vs huincha, y rectitud por gyro.
import sys, math
TICKS_REV=994.0; D_RUEDA=0.065
M_TICK=math.pi*D_RUEDA/TICKS_REV
real=None
if "--real" in sys.argv: real=float(sys.argv[sys.argv.index("--real")+1])
rows=[]
for ln in sys.stdin:
    p=ln.split()
    if len(p)==9 and not ln.startswith("#"):
        try: rows.append((p[0],)+tuple(float(x) for x in p[1:]))
        except ValueError: pass
if len(rows)<4: sys.exit("log insuficiente (%d filas)" % len(rows))
def quieto(r1,r2): return all(abs(r1[i]-r2[i])<3 for i in (1,2,3,4))
# inicio: última fila quieta antes del primer movimiento; fin: última fila (quieta al final)
i0=0
for i in range(1,len(rows)):
    if not quieto(rows[i-1],rows[i]): i0=i-1; break
a,b=rows[i0],rows[-1]
dfl,drl,dfr,drr=(b[i]-a[i] for i in (1,2,3,4))
izq=(dfl+drl)/2*M_TICK; der=(dfr+drr)/2*M_TICK
enc=(izq+der)/2
dyaw=(b[5]-a[5])/1000.0
lidar=[]
if not math.isnan(a[6]) and not math.isnan(b[6]): lidar.append(abs(b[6]-a[6]))
if not math.isnan(a[7]) and not math.isnan(b[7]): lidar.append(abs(b[7]-a[7]))
print("=== TEST 2 m — resultados ===")
print("encoders: total %.3f m  (izq %.3f | der %.3f, diferencia %.1f%%)" % (enc,izq,der,(izq-der)/max(enc,1e-9)*100))
for i,l in enumerate(lidar): print("láser pared %d: %.3f m" % (i+1,l))
if lidar:
    L=sum(lidar)/len(lidar)
    print("factor escala (láser/encoders): %.4f  -> M_PER_TICK corregido: %.6e" % (L/enc,M_TICK*L/enc))
print("giro neto (gyro): %+.1f° %s" % (math.degrees(dyaw),"(recto ✓)" if abs(math.degrees(dyaw))<4 else "(¡curveó!)"))
rot_enc=(der-izq)/0.215
print("rotación según encoders: %+.1f° (vs gyro: coherencia %s)" % (math.degrees(rot_enc),"OK" if abs(rot_enc-dyaw)<math.radians(6) else "REVISAR"))
if real:
    print("huincha: %.3f m -> escala huincha/encoders: %.4f" % (real,real/enc))
