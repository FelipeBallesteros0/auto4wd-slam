// i2c_busca.ino — Due: busca el MPU-9250 en CUALQUIER par de pines digitales con I2C por software
// (pull-ups internos, ~25 kHz). Prueba todos los pares ordenados (sda,scl) de los pines candidatos
// (excluye motores 5-12, encoders 22-29, serial 0/1 y LED 13) y lee WHO_AM_I (0x71/0x73 = MPU-9250).
// También lista qué pines tienen pull-up externo (suben solos tras descargarlos).
const uint8_t CAND[] = {2,3,4,14,15,16,17,18,19,20,21,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,68,69,70,71};
const int NC = sizeof(CAND);
uint8_t SDAp, SCLp;
#define D delayMicroseconds(20)
inline void rel(uint8_t p){ pinMode(p, INPUT_PULLUP); }
inline void lo(uint8_t p){ pinMode(p, OUTPUT); digitalWrite(p, LOW); }
void i2c_start(){ rel(SDAp); rel(SCLp); D; lo(SDAp); D; lo(SCLp); D; }
void i2c_stop(){ lo(SDAp); D; rel(SCLp); D; rel(SDAp); D; }
bool i2c_wbyte(uint8_t b){
  for(int i=7;i>=0;i--){ if(b&(1<<i)) rel(SDAp); else lo(SDAp); D; rel(SCLp); D; lo(SCLp); D; }
  rel(SDAp); delayMicroseconds(80); rel(SCLp); D; bool ack=!digitalRead(SDAp); lo(SCLp); D; return ack;
}
uint8_t i2c_rbyte(){ uint8_t v=0; rel(SDAp); delayMicroseconds(40);
  for(int i=7;i>=0;i--){ rel(SCLp); D; if(digitalRead(SDAp)) v|=(1<<i); lo(SCLp); D; }
  rel(SDAp); D; rel(SCLp); D; lo(SCLp); D; return v; }   // NACK final
bool idle(){ rel(SDAp); rel(SCLp); delayMicroseconds(100); return digitalRead(SDAp) && digitalRead(SCLp); }
// -1 = sin ACK; 0x100 = ACK a la dirección pero falló la lectura; si no, WHO_AM_I
int probe(uint8_t sda, uint8_t scl, uint8_t addr){
  SDAp=sda; SCLp=scl; if(!idle()) return -1;
  i2c_start(); if(!i2c_wbyte(addr<<1)){ i2c_stop(); return -1; }
  bool b=i2c_wbyte(0x75); i2c_start(); bool c=i2c_wbyte((addr<<1)|1);
  int who=0x100; if(b&&c) who=i2c_rbyte(); i2c_stop();
  pinMode(sda,INPUT); pinMode(scl,INPUT);
  return who;
}
bool pullupExterno(uint8_t p){ int h=0;
  for(int i=0;i<10;i++){ lo(p); delayMicroseconds(50); pinMode(p,INPUT); delayMicroseconds(300); h+=digitalRead(p); }
  return h>=8; }
void setup(){ Serial.begin(115200); delay(1500); Serial.println(F("== i2c_busca ==")); }
void loop(){
  Serial.print(F("pines con pull-up externo (20/21 lo tienen en placa):"));
  for(int i=0;i<NC;i++) if(pullupExterno(CAND[i])){ Serial.print(' '); Serial.print(CAND[i]); }
  Serial.println();
  int n=0;
  for(int i=0;i<NC;i++) for(int j=0;j<NC;j++){ if(i==j) continue;
    for(uint8_t a=0x68;a<=0x69;a++){
      int w=probe(CAND[i],CAND[j],a);
      if(w!=-1){ n++; char b[96];
        snprintf(b,sizeof(b),"  ACK sda=%d scl=%d addr=0x%02X who=0x%02X%s", CAND[i],CAND[j],a,w&0xFF,
          (w==0x71||w==0x73)?"  <== MPU-9250!":(w==0x100?" (lectura fallo)":""));
        Serial.println(b); }
    } }
  Serial.print(F("barrido completo: ")); Serial.print(n); Serial.println(F(" respuestas"));
  delay(2500);
}
