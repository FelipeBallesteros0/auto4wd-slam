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

