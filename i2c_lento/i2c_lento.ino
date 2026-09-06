// i2c_lento.ino — ¿es la velocidad? Prueba el MPU en 0x68/0x69 por Wire (20/21) a 100, 50, 20, 10 y
// 5 kHz, y además con I2C por software a ~1 kHz (flancos lentísimos, cables largos no importan).
#include <Wire.h>
const uint32_t VEL[] = {100000, 50000, 20000, 10000, 5000};
// --- I2C por software en 20/21, muy lento ---
#define D delayMicroseconds(250)
inline void rel(uint8_t p){ pinMode(p, INPUT_PULLUP); }
inline void lo(uint8_t p){ pinMode(p, OUTPUT); digitalWrite(p, LOW); }
bool sw_probe(uint8_t addr){
  rel(20); rel(21); D; lo(20); D; lo(21); D;                       // start
  uint8_t b = addr << 1;
  for (int i = 7; i >= 0; i--) { if (b & (1 << i)) rel(20); else lo(20); D; rel(21); D; lo(21); D; }
  rel(20); D; rel(21); D; bool ack = !digitalRead(20); lo(21); D;   // ACK
  lo(20); D; rel(21); D; rel(20); D;                                // stop
  pinMode(20, INPUT); pinMode(21, INPUT);
  return ack;
}
void setup(){ Serial.begin(115200); delay(2000); Serial.println("== i2c_lento =="); }
void loop(){
  for (uint8_t k = 0; k < 5; k++) {
    Wire.begin(); Wire.setClock(VEL[k]);
    Serial.print(VEL[k] / 1000); Serial.print(" kHz: ");
    int n = 0;
    for (byte a = 1; a < 127; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) { Serial.print("0x"); Serial.print(a, HEX); Serial.print(" "); n++; } }
    Wire.beginTransmission(0x68); byte e68 = Wire.endTransmission();
    Wire.beginTransmission(0x69); byte e69 = Wire.endTransmission();
    Serial.print(n ? "" : "nada "); Serial.print(" (0x68 cod="); Serial.print(e68); Serial.print(", 0x69 cod="); Serial.print(e69); Serial.println(")");
    Wire.end();
  }
  Serial.print("software ~1 kHz: 0x68 ack="); Serial.print(sw_probe(0x68)); Serial.print("  0x69 ack="); Serial.println(sw_probe(0x69));
  Serial.println("---");
  delay(2000);
}
