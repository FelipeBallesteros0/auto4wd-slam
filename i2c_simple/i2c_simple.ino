// i2c_simple.ino — lo más básico posible: el escáner I2C clásico de Arduino en Wire (SDA=20, SCL=21)
// y una lectura directa de WHO_AM_I (registro 0x75) en 0x68 y 0x69. Sin trucos.
#include <Wire.h>
void setup() {
  Serial.begin(115200);
  Wire.begin();                 // maestro, 100 kHz por defecto
  delay(2000);
  Serial.println("== i2c_simple ==");
}
void loop() {
  int n = 0;
  Serial.print("scan: ");
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    byte err = Wire.endTransmission();
    if (err == 0) { Serial.print("0x"); Serial.print(a, HEX); Serial.print(" "); n++; }
  }
  if (n == 0) Serial.print("ningun dispositivo");
  Serial.println();
  for (byte a = 0x68; a <= 0x69; a++) {
    Wire.beginTransmission(a);
    Wire.write(0x75);                          // WHO_AM_I
    byte err = Wire.endTransmission(false);    // repeated start
    byte got = Wire.requestFrom((int)a, 1);
    Serial.print("  0x"); Serial.print(a, HEX); Serial.print(": endTransmission="); Serial.print(err);
    Serial.print(" bytes="); Serial.print(got);
    if (got == 1) { Serial.print(" WHO_AM_I=0x"); Serial.print(Wire.read(), HEX); }
    Serial.println();
  }
  delay(2000);
}
