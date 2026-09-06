// imu_diag.ino — Due: ¿llega el MPU-9250 al bus? Cada 1,5 s imprime:
//  - nivel (% en alto tras descargar el pin) de SDA20/SCL21 (pull-up 1,5k en la placa: deben dar
//    100; 0 = corto a GND) y de SDA1/SCL1 (sin pull-up en la placa: 100 = hay un módulo con
//    pull-ups conectado ahí; 0 = al aire).
//  - respuesta del IMU en Wire (20/21) a 100 kHz y 20 kHz, y en Wire1 (SDA1/SCL1) a 20 kHz con
//    pull-ups internos. Sirve para mover cables y ver en qué momento aparece.
#include <Wire.h>
const uint8_t PINS[4]={20,21,70,71};
// Descarga el pin (salida LOW) y lo suelta: si hay pull-up externo/en placa sube a 1; si está
// al aire se queda en 0 (un pin flotante conserva el último nivel por capacidad, por eso se descarga).
int nivel(uint8_t p){ int h=0;
  for(int i=0;i<20;i++){ pinMode(p,OUTPUT); digitalWrite(p,LOW); delayMicroseconds(50);
    pinMode(p,INPUT); delayMicroseconds(300); h+=digitalRead(p); }
  return h*5; }
bool probe(TwoWire&w,uint8_t a,uint8_t*who){
  w.beginTransmission(a); if(w.endTransmission()!=0) return false;
  w.beginTransmission(a); w.write(0x75);
  if(w.endTransmission(false)!=0){ *who=0xEE; return true; }
  if(w.requestFrom((int)a,1)!=1){ *who=0xEF; return true; }
  *who=w.read(); return true;
}
void pullup(uint8_t p){ g_APinDescription[p].pPort->PIO_PUER = g_APinDescription[p].ulPin; }
void addr(String&r,const char*tag,TwoWire&w){
  uint8_t who;
  for(uint8_t a=0x68;a<=0x69;a++) if(probe(w,a,&who)){ r+=" "; r+=tag; r+=":0x"; r+=String(a,HEX); r+=" WHO=0x"; r+=String(who,HEX); }
}
void setup(){ Serial.begin(115200); delay(1500); Serial.println(F("== imu_diag ==")); }
void loop(){
  for(int i=0;i<4;i++) pinMode(PINS[i],INPUT);
  delay(5);
  int n[4]; for(int i=0;i<4;i++) n[i]=nivel(PINS[i]);
  String r="";
  Wire.begin();  Wire.setClock(100000); addr(r,"W100k",Wire);
  Wire.setClock(20000);                 addr(r,"W20k",Wire);
  Wire1.begin(); Wire1.setClock(20000); pullup(70); pullup(71); addr(r,"W1_20k",Wire1);
  char b[200];
  snprintf(b,sizeof(b),"[%lus] alto%%: SDA20=%d SCL21=%d SDA1=%d SCL1=%d | IMU:%s",
    millis()/1000, n[0],n[1],n[2],n[3], r.length()?r.c_str():" (no responde)");
  Serial.println(b);
  delay(1500);
}
