// Escáner I2C Due: bus Wire (SDA=20,SCL=21) y Wire1 (SDA1/SCL1 junto a AREF)
#include <Wire.h>
void scan(TwoWire &w, const char*nombre){
  Serial.print("== bus "); Serial.print(nombre); Serial.println(" ==");
  int n=0;
  for(uint8_t a=8;a<120;a++){
    w.beginTransmission(a);
    if(w.endTransmission()==0){
      Serial.print("  dispositivo en 0x"); Serial.println(a,HEX); n++;
      // si parece MPU, leer WHO_AM_I (0x75)
      w.beginTransmission(a); w.write(0x75);
      if(w.endTransmission(false)==0 && w.requestFrom((int)a,1)==1){
        Serial.print("    WHO_AM_I=0x"); Serial.println(w.read(),HEX);
      }
    }
  }
  if(!n) Serial.println("  (nada)");
}
void setup(){ Serial.begin(115200); Wire.begin(); Wire.setClock(100000); Wire1.begin(); Wire1.setClock(100000); delay(500); }
void loop(){ Serial.println("--- escaneo ---"); scan(Wire,"Wire (20/21)"); scan(Wire1,"Wire1 (SDA1/SCL1)"); delay(3000); }
