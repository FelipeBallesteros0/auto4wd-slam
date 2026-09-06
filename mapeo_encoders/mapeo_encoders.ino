/*
 * mapeo_encoders.ino  —  Arduino Due (robot 4WD, RPi "auto4wd")
 * Objetivo: descubrir qué encoder corresponde a cada motor (rueda),
 *           su sentido de giro y si el encoder es cuadratura real.
 *
 * Puentes H: IN1/IN2 ambos PWM. Sentido = cuál pin del par lleva el PWM.
 *   M1=(5,6)  M2=(7,8)  M3=(9,10)  M4=(11,12)
 * Encoders: 2 canales por rueda (pares consecutivos), con interrupción.
 *   E1=(22,23) E2=(24,25) E3=(26,27) E4=(28,29)
 *
 * Con el robot ELEVADO (ruedas al aire). Serial 115200 (Puerto de Programación).
 */

#include <Wire.h>
const uint8_t MPIN[4][2] = {{5,6},{7,8},{9,10},{11,12}};
const uint8_t EPIN[4][2] = {{22,23},{24,25},{26,27},{28,29}};

volatile long  edge[8];        // flancos por canal:  idx = enc*2 + canal(0=A,1=B)
volatile long  quadPos[4];     // posición con signo (decodificación cuadratura)
volatile uint8_t encPrev[4];   // último estado AB por encoder (a<<1|b)

// Tabla de transición de cuadratura (Gray): idx [previo][actual] -> -1/0/+1
const int8_t QDEC[4][4] = {
  { 0, +1, -1,  0},
  {-1,  0,  0, +1},
  {+1,  0,  0, -1},
  { 0, -1, +1,  0}
};

const int SPEED  = 80;    // PWM bajo (0-255) para pruebas suaves
const int RUN_MS = 1200;  // ms girando en cada sentido
const int SETTLE = 350;   // ms de reposo entre pasos

inline void handle(uint8_t k) {
  uint8_t a = digitalRead(EPIN[k][0]);
  uint8_t b = digitalRead(EPIN[k][1]);
  uint8_t s = (a << 1) | b;
  uint8_t p = encPrev[k];
  if (((p >> 1) & 1) != a) edge[k*2 + 0]++;   // cambió canal A
  if ((p & 1)        != b) edge[k*2 + 1]++;   // cambió canal B
  quadPos[k] -= QDEC[p][s];   // signo invertido: ADELANTE del robot = cuenta positiva
  encPrev[k] = s;
}
void isr0(){ handle(0); }
void isr1(){ handle(1); }
void isr2(){ handle(2); }
void isr3(){ handle(3); }
void (*ISRS[4])() = { isr0, isr1, isr2, isr3 };

void motorSet(uint8_t m, int spd) {          // spd: -255..255  (+ = ADELANTE del robot)
  uint8_t a = MPIN[m][0], b = MPIN[m][1];
  // Confirmado 2026-08-30: PWM en el primer pin movia el robot HACIA ATRAS.
  if (spd >= 0) { analogWrite(b, spd);  analogWrite(a, 0); }
  else          { analogWrite(b, 0);    analogWrite(a, -spd); }
}
void motorStop(uint8_t m){ analogWrite(MPIN[m][0], 0); analogWrite(MPIN[m][1], 0); }
void allStop(){ for (uint8_t m=0;m<4;m++) motorStop(m); }

void resetCounts(){
  noInterrupts();
  for (int i=0;i<8;i++) edge[i]=0;
  for (int k=0;k<4;k++){ quadPos[k]=0;
    encPrev[k] = (digitalRead(EPIN[k][0])<<1) | digitalRead(EPIN[k][1]); }
  interrupts();
}

int  mapMotorEnc[4];   // encoder detectado por motor (-1 si nada)
long mapFwdPos[4];     // posDir+ del encoder detectado (para el resumen)
long snapEdge[8]; long snapPos[4];
void snapshot(){ noInterrupts(); for(int i=0;i<8;i++) snapEdge[i]=edge[i];
                 for(int k=0;k<4;k++) snapPos[k]=quadPos[k]; interrupts(); }

void probeMotor(uint8_t m){
  Serial.print(F("\n=== Motor M")); Serial.print(m+1);
  Serial.print(F("  pines(")); Serial.print(MPIN[m][0]); Serial.print(',');
  Serial.print(MPIN[m][1]); Serial.println(F(") ==="));

  // --- sentido + ---
  resetCounts(); motorSet(m, SPEED); delay(RUN_MS); motorStop(m); delay(SETTLE);
  snapshot();
  long fEdge[8]; long fPos[4];
  for(int i=0;i<8;i++) fEdge[i]=snapEdge[i]; for(int k=0;k<4;k++) fPos[k]=snapPos[k];

  // --- sentido - ---
  resetCounts(); motorSet(m, -SPEED); delay(RUN_MS); motorStop(m); delay(SETTLE);
  snapshot();
  long rEdge[8]; long rPos[4];
  for(int i=0;i<8;i++) rEdge[i]=snapEdge[i]; for(int k=0;k<4;k++) rPos[k]=snapPos[k];

  // tabla de actividad por encoder
  Serial.println(F("  Enc  flancosA+  flancosB+  posDir+   flancosA-  flancosB-  posDir-"));
  int best=-1; long bestTot=0;
  for(int k=0;k<4;k++){
    long tot = fEdge[k*2]+fEdge[k*2+1]+rEdge[k*2]+rEdge[k*2+1];
    if(tot>bestTot){ bestTot=tot; best=k; }
    char buf[96];
    snprintf(buf,sizeof(buf),"  E%d   %7ld  %8ld  %7ld    %7ld  %8ld  %7ld",
      k+1, fEdge[k*2], fEdge[k*2+1], fPos[k], rEdge[k*2], rEdge[k*2+1], rPos[k]);
    Serial.println(buf);
  }

  if(best<0 || bestTot<20){
    mapMotorEnc[m]=-1;
    Serial.println(F("  >> SIN MOVIMIENTO detectado (¿motor no gira? ¿alimentación de motores? ¿cableado?)"));
    return;
  }
  mapMotorEnc[m]=best; mapFwdPos[m]=fPos[best];
  long fA=fEdge[best*2], fB=fEdge[best*2+1];
  bool cuad = (fA>5 && fB>5);                 // ambos canales activos -> cuadratura probable
  bool balance = cuad && ( (fA> fB? fA:fB) <= 3*(fA<fB? fA:fB) );
  Serial.print(F("  >> Motor M")); Serial.print(m+1);
  Serial.print(F("  ->  Encoder E")); Serial.print(best+1);
  Serial.print(F(" (pines ")); Serial.print(EPIN[best][0]); Serial.print(',');
  Serial.print(EPIN[best][1]); Serial.print(')');
  Serial.print(F("  | sentido+ = ")); Serial.print(fPos[best]>=0?F("cuenta SUBE"):F("cuenta BAJA"));
  Serial.print(F("  | cuadratura: "));
  if(cuad && balance) Serial.println(F("SÍ (2 canales balanceados)"));
  else if(cuad)       Serial.println(F("parcial (2 canales, desbalance)"));
  else                Serial.println(F("NO (solo 1 canal cuenta)"));
}

// ---- escáner I2C (comando 'i'): ¿está el MPU-9250 en el bus? ----
void i2cScanBus(TwoWire &w, const char*nombre){
  Serial.print(F("  bus ")); Serial.print(nombre); Serial.print(F(": "));
  int n=0;
  for(uint8_t a=8;a<120;a++){
    w.beginTransmission(a);
    if(w.endTransmission()==0){
      n++; Serial.print(F("0x")); Serial.print(a,HEX);
      w.beginTransmission(a); w.write(0x75);
      if(w.endTransmission(false)==0 && w.requestFrom((int)a,1)==1){
        Serial.print(F("(WHO_AM_I=0x")); Serial.print(w.read(),HEX); Serial.print(')');
      }
      Serial.print(' ');
    }
  }
  if(!n) Serial.print(F("(nada)"));
  Serial.println();
}
void i2cScan(){
  Serial.println(F("[I2C] escaneo (MPU-9250 esperado en 0x68 o 0x69, WHO_AM_I 0x71/0x73):"));
  i2cScanBus(Wire,"Wire SDA=20/SCL=21");
  i2cScanBus(Wire1,"Wire1 SDA1/SCL1");
}

void printSummary(){
  Serial.println(F("\n########## RESUMEN MAPEO rueda<->encoder ##########"));
  Serial.println(F("Motor(pines)   ->  Encoder(pines)   sentido+"));
  for(int m=0;m<4;m++){
    Serial.print(F("M")); Serial.print(m+1);
    Serial.print('(');Serial.print(MPIN[m][0]);Serial.print(',');Serial.print(MPIN[m][1]);Serial.print(')');
    Serial.print(F("      ->  "));
    int k=mapMotorEnc[m];
    if(k<0){ Serial.println(F("(ninguno)")); continue; }
    Serial.print(F("E"));Serial.print(k+1);
    Serial.print('(');Serial.print(EPIN[k][0]);Serial.print(',');Serial.print(EPIN[k][1]);Serial.print(')');
    Serial.print(F("     "));
    Serial.println(mapFwdPos[m]>=0?F("+"):F("-"));
  }
  // avisar duplicados
  for(int a=0;a<4;a++) for(int b=a+1;b<4;b++)
    if(mapMotorEnc[a]>=0 && mapMotorEnc[a]==mapMotorEnc[b]){
      Serial.print(F("¡OJO! M"));Serial.print(a+1);Serial.print(F(" y M"));Serial.print(b+1);
      Serial.println(F(" apuntan al mismo encoder — revisar (acoplamiento mecánico o cableado)."));
    }
  Serial.println(F("###################################################"));
  Serial.println(F("(envía 'r' por serial para repetir el mapeo)"));
}

void runMapping(){
  Serial.println(F("\n>>> Iniciando mapeo. Robot ELEVADO. Girando cada rueda a PWM baja..."));
  allStop(); delay(300);
  for(uint8_t m=0;m<4;m++) probeMotor(m);
  allStop();
  printSummary();
}

void setup(){
  Serial.begin(115200);
  for(uint8_t m=0;m<4;m++){ pinMode(MPIN[m][0],OUTPUT); pinMode(MPIN[m][1],OUTPUT); }
  allStop();
  for(uint8_t k=0;k<4;k++){
    pinMode(EPIN[k][0],INPUT_PULLUP); pinMode(EPIN[k][1],INPUT_PULLUP);
    encPrev[k]=(digitalRead(EPIN[k][0])<<1)|digitalRead(EPIN[k][1]);
    attachInterrupt(digitalPinToInterrupt(EPIN[k][0]), ISRS[k], CHANGE);
    attachInterrupt(digitalPinToInterrupt(EPIN[k][1]), ISRS[k], CHANGE);
  }
  for(int m=0;m<4;m++) mapMotorEnc[m]=-1;
  unsigned long t0=millis();
  while(!Serial && millis()-t0<3000) { }
  delay(1500);
  Serial.println(F("\n== mapeo_encoders (Arduino Due) listo =="));
  Serial.println(F("comandos: r=mapeo  a=todas adelante 4s  1-4=solo ese motor 2s  m=resumen  z=cero  i=escaneo I2C"));
  Wire.begin(); Wire.setClock(100000); Wire1.begin(); Wire1.setClock(100000);
  i2cScan();
}

void printLive(){
  static long pe[8]={0}; static long pp[4]={0};
  long e[8]; long q[4];
  noInterrupts(); for(int i=0;i<8;i++) e[i]=edge[i]; for(int k=0;k<4;k++) q[k]=quadPos[k]; interrupts();
  bool ch=false; for(int i=0;i<8;i++) if(e[i]!=pe[i]) ch=true;
  if(!ch) return;
  char buf[120];
  snprintf(buf,sizeof(buf),"CNT E1:%ld/%ld p%ld  E2:%ld/%ld p%ld  E3:%ld/%ld p%ld  E4:%ld/%ld p%ld",
    e[0],e[1],q[0], e[2],e[3],q[1], e[4],e[5],q[2], e[6],e[7],q[3]);
  Serial.println(buf);
  for(int i=0;i<8;i++) pe[i]=e[i]; for(int k=0;k<4;k++) pp[k]=q[k];
}
void loop(){
  static unsigned long last=0;
  if(Serial.available()){
    char c=Serial.read();
    if(c=='r'||c=='R') runMapping();       // repetir mapeo con motores
    if(c=='z'||c=='Z'){ resetCounts(); Serial.println(F("[contadores a cero]")); }
    if(c=='m'||c=='M') printSummary();     // resumen bajo demanda
    if(c=='i'||c=='I') i2cScan();          // ¿aparece el MPU en el bus?
    if(c=='a'||c=='A'){                    // todas adelante 4 s
      Serial.println(F("[TODAS ADELANTE 4 s, PWM 90]"));
      resetCounts();
      for(uint8_t m2=0;m2<4;m2++) motorSet(m2, 90);
      delay(4000); allStop(); delay(300); snapshot();
      Serial.println(F("posiciones tras 4 s (deben ser todas POSITIVAS y parecidas):"));
      char b2[96];
      snprintf(b2,sizeof(b2),"  E1:%ld  E2:%ld  E3:%ld  E4:%ld", snapPos[0],snapPos[1],snapPos[2],snapPos[3]);
      Serial.println(b2);
    }
    if(c>='1'&&c<='4'){                    // un solo motor adelante 2 s
      uint8_t m2=c-'1';
      Serial.print(F("[gira SOLO M")); Serial.print(m2+1); Serial.println(F(" adelante 2 s]"));
      motorSet(m2, 90); delay(2000); motorStop(m2);
    }
  }
  if(millis()-last>500){ last=millis(); printLive(); }  // monitor en vivo (gira ruedas a mano)
}
