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
// ¿Puede el Due mover físicamente cada pin? (driver dañado = no baja a 0)
void testDrive(uint8_t p, char*out){
  pinMode(p,OUTPUT); digitalWrite(p,LOW); delayMicroseconds(50); int a=digitalRead(p);
  digitalWrite(p,HIGH); delayMicroseconds(50); int b=digitalRead(p);
  pinMode(p,INPUT); delayMicroseconds(300); int c=digitalRead(p);
  sprintf(out,"pin%d: bajo=%d alto=%d suelto=%d%s", p,a,b,c,(a==0&&b==1&&c==1)?" OK":" <-- RARO");
}
// ¿El bus responde rápido con NACK (bus vivo, nadie contesta) o se queda esperando (bus muerto)?
void testWire(){
  Wire.begin(); Wire.setClock(100000);
  Wire.beginTransmission(0x68); unsigned long t0=micros(); uint8_t e=Wire.endTransmission(); unsigned long dt=micros()-t0;
  Serial.print(F("  Wire 0x68: codigo=")); Serial.print(e); Serial.print(F(" en ")); Serial.print(dt); Serial.print(F(" us -> "));
  if(e==0) Serial.println(F("RESPONDE"));
  else if(dt<2000) Serial.println(F("NACK inmediato: el bus funciona, el modulo no contesta"));
  else Serial.println(F("TIMEOUT: el bus no se mueve (SCL/SDA trabados o pin danado)"));
}
// ¿SDA y SCL están puenteados entre sí (soldadura)? Bajo uno y miro el otro.
void testPuente(){
  pinMode(21,INPUT); pinMode(20,OUTPUT); digitalWrite(20,LOW); delayMicroseconds(100); int a=digitalRead(21);
  pinMode(20,INPUT); pinMode(21,OUTPUT); digitalWrite(21,LOW); delayMicroseconds(100); int b=digitalRead(20);
  pinMode(20,INPUT); pinMode(21,INPUT);
  Serial.print(F("  puente SDA-SCL: bajo20->21=")); Serial.print(a); Serial.print(F(" bajo21->20=")); Serial.print(b);
  Serial.println((a==1&&b==1)?F("  -> NO hay puente"):F("  -> ¡SDA y SCL UNIDOS! (soldadura)"));
}
// Recuperación de bus I2C: 10 pulsos de SCL con SDA suelto + STOP (libera un esclavo colgado con SDA en bajo)
void busRecover(){
  pinMode(20,INPUT_PULLUP); pinMode(21,OUTPUT);
  for(int i=0;i<10;i++){ digitalWrite(21,LOW); delayMicroseconds(10); digitalWrite(21,HIGH); delayMicroseconds(10); }
  pinMode(20,OUTPUT); digitalWrite(20,LOW); delayMicroseconds(10); digitalWrite(21,HIGH); delayMicroseconds(10);
  pinMode(20,INPUT_PULLUP); delayMicroseconds(10); pinMode(20,INPUT); pinMode(21,INPUT);
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
  char t[64]; testDrive(20,t); Serial.print(F("  ")); Serial.print(t); testDrive(21,t); Serial.print(F(" | ")); Serial.println(t);
  testWire(); testPuente();
  busRecover(); Serial.print(F("  tras recuperar bus ->")); testWire();
  delay(1500);
}
