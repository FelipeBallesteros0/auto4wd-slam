# Mapeo rueda ↔ encoder — Arduino Due robot 4WD (2026-08-30)

Método: robot elevado, PWM 80/255, 1,2 s por sentido, conteo de flancos por interrupción.
Corrida limpia (tras arreglar un cable de motor suelto que ensuciaba M3).

| Motor | Pines PWM (IN1,IN2) | Encoder | Pines encoder (A,B) | PWM en IN1 (“+”) | Cuadratura |
|-------|--------------------|---------|--------------------|------------------|-----------|
| M1 | 5, 6   | E4 | 28, 29 | cuenta SUBE | SÍ (967/965 ida, 960/962 vuelta) |
| M2 | 7, 8   | E3 | 26, 27 | cuenta SUBE | SÍ (1059/1059, 1096/1097) |
| M3 | 9, 10  | E2 | 24, 25 | cuenta SUBE | SÍ (1092/1092, 1068/1068) |
| M4 | 11, 12 | E1 | 22, 23 | cuenta SUBE | SÍ (1090/1090, 1085/1084) |

- El orden está **invertido**: M1↔E4, M2↔E3, M3↔E2, M4↔E1.
- Los 4 son encoders de cuadratura reales (2 canales balanceados, avance neto coherente x4).
- Convención de signo uniforme: PWM en el primer pin del par ⇒ posición sube; en el segundo ⇒ baja.
- Velocidad de referencia: ~2100–2200 cuentas x4 en 1,2 s a PWM 80.
- PENDIENTE: qué rueda física (DI/DD/TI/TD) es cada M — se determina viendo el orden en que giran (M1→M2→M3→M4).
- PENDIENTE: cuentas por vuelta de rueda (calibrar con una vuelta manual o ficha del motorreductor).

## Polaridad definitiva (confirmada 2026-08-30)
- Con PWM en el **primer** pin del par, el robot iba **HACIA ATRÁS** (confirmado por Felipe viendo las 4 ruedas).
- **Convención final del firmware (sketch v4): ADELANTE = PWM en el SEGUNDO pin del par** (6, 8, 10, 12) y el decodificador de cuadratura va con signo invertido ⇒ **adelante = cuentas positivas** en los 4 encoders.
- Verificado con "todas adelante 4 s": E1 +8105 · E2 +8119 · E3 +8337 · E4 +7231 (M1/E4 ~10 % más lento, emparejar con PID).
- Verificación individual: cada comando 1–4 movió exactamente su encoder mapeado.
## TABLA FINAL COMPLETA (confirmada por Felipe 2026-08-30)
Posiciones vistas desde arriba ("superior" = delantera, "inferior" = trasera):

| Motor | Pin atrás | Pin ADELANTE | Encoder | Pines enc (A,B) | Rueda física |
|-------|-----------|--------------|---------|-----------------|--------------|
| M1 | 5  | **6**  | E4 | 28, 29 | **Delantera IZQUIERDA** (superior izq) |
| M2 | 7  | **8**  | E3 | 26, 27 | **Trasera IZQUIERDA** (inferior izq) |
| M3 | 9  | **10** | E2 | 24, 25 | **Delantera DERECHA** (superior der) |
| M4 | 11 | **12** | E1 | 22, 23 | **Trasera DERECHA** (inferior der) |

- Lado IZQUIERDO = M1+M2 · lado DERECHO = M3+M4 (útil para skid-steer).
- Confirmado visualmente: con la convención v4 (PWM en pin "ADELANTE"), las 4 ruedas avanzan y los 4 encoders cuentan positivo.


## Re-cableado 2026-09-06 (Felipe cambió el orden de pines PWM y de encoders)
Método: (1) sketch `mapeo_encoders` comando `r` → qué encoder responde a cada par PWM y con qué
signo; (2) ruedas giradas **a mano hacia adelante** en orden DI→TI→DD→TD con el monitor `CNT` →
qué encoder es cada rueda y su signo al avanzar; (3) firmware nuevo + `test_adelante.py` (V 70 70,
2 s): fl rl fr rr = +3506 +3610 +3573 +3550 ⇒ los 4 positivos y parejos.

| Rueda | PWM {atrás, ADELANTE} | Encoder (A,B) | Signo bruto al avanzar | ENC_SIGN |
|---|---|---|---|---|
| Delantera izquierda | {12, **11**} | E1 (22,23) | + | +1 |
| Trasera izquierda   | { 8, **7**}  | E3 (26,27) | + | +1 |
| Delantera derecha   | { 9, **10**} | E2 (24,25) | − | −1 |
| Trasera derecha     | { 5, **6**}  | E4 (28,29) | − | −1 |

- El firmware (v3.2) ahora indexa **por rueda** en el orden de la telemetría (0=DI 1=TI 2=DD 3=TD);
  desapareció `ENC_DE_MOTOR` y entró `ENC_SIGN` (el lado derecho cuenta al revés por espejo).
- Herramientas en la Pi (`~/teleop/`): `serial_cmd.py <chars> <seg> [espera] [pausa]` para hablar
  con el sketch de mapeo (`r`, `i`=escaneo I2C, `1-4`, `a`, `z`), `test_adelante.py [pwm] [seg]`
  para verificar el firmware teleop, `enc_live.log` del monitor a mano.
- Fallas encontradas por el camino: pin 22 (canal A de E1) suelto; motores de 5/6 y 11/12 sin
  girar hasta reasentar cables; el Due se desconectaba del USB ~20 veces mientras se manipulaba
  el robot (cable USB flojo). **IMU: sigue sin aparecer en I2C (SDA=20/SCL=21).**
