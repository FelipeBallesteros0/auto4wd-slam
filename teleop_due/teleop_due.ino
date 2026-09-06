/*
 * teleop_due.ino — Robot 4WD (RPi manda por serial, Due ejecuta)
 * Protocolo (115200, línea terminada en \n):
 *   RPi -> Due:  "V <izq> <der>"  PWM con signo -255..255 por lado
 *                "S"              stop inmediato
 *   Due -> RPi:  "T <fl> <rl> <fr> <rr>"  posiciones de encoder (x4, adelante=+) cada 100 ms
 *                "R listo"        al arrancar
 * Watchdog: sin comando válido por 300 ms => motores a cero.
 * IMU MPU-9250 (I2C 20/21, 3.3V): gyro Z -> rumbo integrado (yaw). Telemetría:
 *   "T <fl> <rl> <fr> <rr> <yaw_mrad> <gz_mrad_s>"  (los 2 últimos solo si IMU OK)
 * Re-cableado 2026-09-06 (Felipe cambió pines PWM y encoders). Tablas POR RUEDA, en el
 * mismo orden que la telemetría: 0=del.izq 1=tras.izq 2=del.der 3=tras.der.
 *   Rueda      PWM {atrás, ADELANTE}   Encoder (A,B)   signo al avanzar (medido a mano)
 *   del.izq    {12, 11}                 (22,23)         +
 *   tras.izq   { 8,  7}                 (26,27)         +
 *   del.der    { 9, 10}                 (24,25)         −   (lado derecho espejado)
 *   tras.der   { 5,  6}                 (28,29)         −
 * ENC_SIGN compensa el espejo: los 4 encoders cuentan + al avanzar.
 */
const uint8_t MPIN[4][2] = {{12,11},{8,7},{9,10},{5,6}};   // {atras, adelante} por rueda
const uint8_t EPIN[4][2] = {{22,23},{26,27},{24,25},{28,29}};
const int8_t  ENC_SIGN[4] = {+1,+1,-1,-1};                 // adelante = + en los 4

volatile long quadPos[4]; volatile uint8_t encPrev[4];
// Lectura DIRECTA de puertos en la ISR: digitalRead era tan lento que a velocidad
// de marcha se perdían flancos (~10% de subconteo, test huincha 2026-09-01).
Pio* encPort[4][2]; uint32_t encMask[4][2];
const int8_t QDEC[4][4]={{0,+1,-1,0},{-1,0,0,+1},{+1,0,0,-1},{0,-1,+1,0}};
// Decodificación por MUESTREO a 20 kHz (timer TC0): las interrupciones por flanco
// del Due pierden cuentas a velocidad (−10% bruto, −7.8% con ISR rápida; test
// huincha 2026-09-01). Muestreando a 20 kHz (flancos máx ~3 kHz/canal) no se
// pierde ninguno, a cualquier velocidad.
void encTimerSetup(){
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC0);
  TC_Configure(TC0,0, TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC|TC_CMR_TCCLKS_TIMER_CLOCK2); // MCK/8=10.5MHz
  TC_SetRC(TC0,0, 525);                                    // 10.5e6/525 = 20 kHz
  TC0->TC_CHANNEL[0].TC_IER=TC_IER_CPCS;
  TC0->TC_CHANNEL[0].TC_IDR=~TC_IER_CPCS;
  NVIC_EnableIRQ(TC0_IRQn);
  TC_Start(TC0,0);
}
void TC0_Handler(){
  TC_GetStatus(TC0,0);
  for(uint8_t k=0;k<4;k++){
    uint8_t a=(encPort[k][0]->PIO_PDSR & encMask[k][0])?1:0;
    uint8_t b=(encPort[k][1]->PIO_PDSR & encMask[k][1])?1:0;
    uint8_t st=(a<<1)|b; quadPos[k]-=ENC_SIGN[k]*QDEC[encPrev[k]][st]; encPrev[k]=st;  // adelante = +
  }
}

void motorSet(uint8_t m,int spd){          // + = adelante
  spd=constrain(spd,-255,255);
  if(spd>=0){ analogWrite(MPIN[m][1],spd); analogWrite(MPIN[m][0],0); }
  else      { analogWrite(MPIN[m][1],0);   analogWrite(MPIN[m][0],-spd); }
}
void allStop(){ for(uint8_t m=0;m<4;m++) motorSet(m,0); }

unsigned long lastCmd=0, lastTel=0, lastBlink=0;
int curL=0, curR=0; bool moving=false;
char line[40]; uint8_t ll=0;

// ---------- IMU MPU-9250 ----------
const uint8_t IMU_ADDRS[2] = {0x68, 0x69};
uint8_t imuAddr=0; bool imuOk=false; uint8_t imuWho=0;
float gzBias=0.0f;               // LSB
volatile float yawRad=0.0f;      // rumbo integrado
float gzRad=0.0f;                // velocidad angular actual (rad/s)
unsigned long imuLastUs=0, imuStillMs=0, imuRetryMs=0, imuDownMs=0, imuReadUs=0;
uint16_t imuNanSeguidas=0, imuCaidas=0; uint32_t imuGlitches=0;
const unsigned long IMU_PERIODO_US=5000;    // 200 Hz = tasa de muestreo del MPU; leer más rápido solo ocupa el bus
const unsigned long IMU_REINTENTO_MS=50;    // reintento rápido: si el chip se reinició tarda ~100 ms en volver; un hueco corto no rompe el mapa
const float GYRO_LSB_DPS = 131.0f;   // ±250 dps

// ---- I2C por SOFTWARE en 20/21 (2026-09-06) ----
// La TWI por hardware del SAM3X se quedaba colgada tras un glitch: cada transacción agotaba
// ~112 ms de timeout de la librería Wire (huecos de 115/337/397 ms = 1/3 timeouts) hasta
// reiniciar el periférico. Bit-bang: ninguna espera al esclavo, un fallo se ve en ~100 us.
// Pull-ups: los 1,5 k de la placa en 20/21. ~60-80 kHz efectivos.
#define I2C_D delayMicroseconds(4)
static inline void sdaRel(){ pinMode(20,INPUT); }
static inline void sdaLo(){ pinMode(20,OUTPUT); digitalWrite(20,LOW); }
static inline void sclRel(){ pinMode(21,INPUT); }
static inline void sclLo(){ pinMode(21,OUTPUT); digitalWrite(21,LOW); }
static inline uint8_t sdaIn(){ return (g_APinDescription[20].pPort->PIO_PDSR & g_APinDescription[20].ulPin)?1:0; }
static inline uint8_t sclIn(){ return (g_APinDescription[21].pPort->PIO_PDSR & g_APinDescription[21].ulPin)?1:0; }
void swStart(){ sdaRel(); sclRel(); I2C_D; sdaLo(); I2C_D; sclLo(); I2C_D; }
void swStop(){ sdaLo(); I2C_D; sclRel(); I2C_D; sdaRel(); I2C_D; }
bool swWrite(uint8_t b){
  for(int i=7;i>=0;i--){ if(b&(1<<i)) sdaRel(); else sdaLo(); I2C_D; sclRel(); I2C_D; sclLo(); I2C_D; }
  sdaRel(); I2C_D; sclRel(); I2C_D; bool ack=!sdaIn(); sclLo(); I2C_D; return ack;
}
uint8_t swRead(bool ack){
  uint8_t v=0; sdaRel();
  for(int i=7;i>=0;i--){ sclRel(); I2C_D; if(sdaIn()) v|=(1<<i); sclLo(); I2C_D; }
  if(ack) sdaLo(); else sdaRel(); I2C_D; sclRel(); I2C_D; sclLo(); I2C_D; sdaRel(); return v;
}
bool i2cLibre(){ return sdaIn() && sclIn(); }
// Recuperación: 10 pulsos de SCL con SDA suelto + STOP (resincroniza un esclavo a mitad de byte)
void i2cRecuperar(){
  sdaRel(); for(int i=0;i<10;i++){ sclLo(); delayMicroseconds(10); sclRel(); delayMicroseconds(10); }
  swStop();
}
bool imuWrite(uint8_t reg, uint8_t val){
  if(!i2cLibre()) return false;
  swStart(); bool ok=swWrite(imuAddr<<1) && swWrite(reg) && swWrite(val); swStop(); return ok;
}
bool imuRead(uint8_t reg, uint8_t n, uint8_t*buf){
  if(!i2cLibre()) return false;
  swStart(); bool ok=swWrite(imuAddr<<1) && swWrite(reg);
  if(ok){ swStart(); ok=swWrite((imuAddr<<1)|1); }
  if(!ok){ swStop(); return false; }
  for(uint8_t i=0;i<n;i++) buf[i]=swRead(i+1<n);
  swStop(); return true;
}
bool imuInit(){
  for(uint8_t k=0;k<5 && !i2cLibre();k++) i2cRecuperar();
  if(!i2cLibre()) return false;            // sigue trabado: reintento en 50 ms
  i2cRecuperar();
  for(uint8_t k=0;k<2;k++){
    imuAddr=IMU_ADDRS[k]; uint8_t who=0;
    if(imuRead(0x75,1,&who) && who!=0x00 && who!=0xFF){
      imuWho=who;
      // Si solo se colgó el bus (no se reinició el chip), PWR_MGMT_1 sigue en 0x01: nada que
      // reconfigurar, y la recuperación tarda ~1 ms en vez de 50 (2026-09-06).
      uint8_t pm=0xFF; if(imuRead(0x6B,1,&pm) && pm==0x01) return true;
      imuWrite(0x6B,0x01);      // reloj PLL, salir de sleep
      delay(50);
      imuWrite(0x1A,0x03);      // DLPF 41 Hz
      imuWrite(0x1B,0x00);      // gyro ±250 dps
      imuWrite(0x19,0x04);      // 200 Hz
      return true;
    }
  }
  return false;
}
float imuReadGzLSB(){ uint8_t b[2]; if(!imuRead(0x47,2,b)) return NAN;
  return (float)((int16_t)((b[0]<<8)|b[1])); }
void imuCalibrate(){             // robot quieto ~2 s
  float acc=0; int n=0;
  for(int i=0;i<200;i++){ float v=imuReadGzLSB(); if(!isnan(v)){acc+=v;n++;} delay(10); }
  if(n>50) gzBias=acc/n;
}
void imuUpdate(){
  unsigned long nowUs=micros();
  if(!imuOk){
    // reintento en caliente (inmediato y luego cada 250 ms) con recuperación de bus.
    // NO recalibrar: conserva el sesgo bueno del arranque (recalibrar aquí, con el módulo
    // posiblemente manipulado/en movimiento, metía deriva — visto 2026-09-03). Solo
    // recalibra si lleva >30 s caído Y quieto.
    if(millis()-imuRetryMs>IMU_REINTENTO_MS){
      imuRetryMs=millis();
      if(imuInit()){
        bool recalibrar = (millis()-imuDownMs>30000) && curL==0 && curR==0;
        if(recalibrar){ delay(300); imuCalibrate(); }
        imuLastUs=micros(); imuReadUs=imuLastUs; imuNanSeguidas=0; imuOk=true;
        Serial.print(F("R IMU=OK 0x")); Serial.print(imuWho,HEX);
        Serial.print(F(" tras ")); Serial.print(millis()-imuDownMs); Serial.print(F(" ms"));
        Serial.println(recalibrar?F(" (hotplug+recal)"):F(" (hotplug)"));
      }
    }
    return;
  }
  if(nowUs-imuReadUs<IMU_PERIODO_US) return;   // 200 Hz: el bus queda libre el 98% del tiempo
  imuReadUs=nowUs;
  float v=imuReadGzLSB();
  if(isnan(v)){ i2cRecuperar(); v=imuReadGzLSB(); imuGlitches++; }   // glitch: resincronizar y reintentar (~300 us)
  nowUs=micros();
  if(isnan(v)){
    if(++imuNanSeguidas>=3){ imuOk=false; imuRetryMs=0; imuDownMs=millis(); imuCaidas++;
      Serial.print(F("R IMU=CAIDO #")); Serial.print(imuCaidas); Serial.println(F(" (mantengo rumbo, reintento)")); }
    return;
  }
  imuNanSeguidas=0;
  float dt=(nowUs-imuLastUs)/1e6f; imuLastUs=nowUs;
  if(dt<=0 || dt>0.2f) return;
  float gzDps=(v-gzBias)/GYRO_LSB_DPS;    // sin invertir: modulo nuevo montado con Z hacia ARRIBA (giro 90° izq = +1571 mrad, validado 2026-09-06)
  gzRad=gzDps*0.0174533f;
  yawRad+=gzRad*dt;
  // re-cero lento del sesgo con robot quieto y giro casi nulo
  if(curL==0 && curR==0 && fabs(gzDps)<1.0f){
    if(imuStillMs==0) imuStillMs=millis();
    else if(millis()-imuStillMs>3000) gzBias=0.999f*gzBias+0.001f*v;
  } else imuStillMs=0;
}


void applyLR(int l,int r){
  curL=l; curR=r; moving=(l!=0||r!=0);
  motorSet(0,l); motorSet(1,l);   // izquierda: del.izq tras.izq
  motorSet(2,r); motorSet(3,r);   // derecha:   del.der tras.der
}
void procLine(){
  line[ll]=0;
  if(line[0]=='V'){
    int l,r;
    if(sscanf(line,"V %d %d",&l,&r)==2){ applyLR(l,r); lastCmd=millis(); }
  } else if(line[0]=='S'){ applyLR(0,0); lastCmd=millis(); }
  ll=0;
}
void setup(){
  Serial.begin(115200);
  for(uint8_t m=0;m<4;m++){ pinMode(MPIN[m][0],OUTPUT); pinMode(MPIN[m][1],OUTPUT); }
  allStop(); pinMode(13,OUTPUT);
  for(uint8_t k=0;k<4;k++){
    pinMode(EPIN[k][0],INPUT_PULLUP); pinMode(EPIN[k][1],INPUT_PULLUP);
    for(uint8_t c=0;c<2;c++){
      encPort[k][c]=g_APinDescription[EPIN[k][c]].pPort;
      encMask[k][c]=g_APinDescription[EPIN[k][c]].ulPin;
    }
    encPrev[k]=(digitalRead(EPIN[k][0])<<1)|digitalRead(EPIN[k][1]);
  }
  encTimerSetup();
  imuOk=imuInit();
  if(imuOk){ delay(800); imuCalibrate(); imuLastUs=micros(); }
  delay(300);
  Serial.print(F("R listo IMU="));
  if(imuOk){ Serial.print(F("OK 0x")); Serial.println(imuWho,HEX); }
  else Serial.println(F("NO"));
}
void loop(){
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'||c=='\r'){ if(ll) procLine(); }
    else if(ll<sizeof(line)-1) line[ll++]=c;
  }
  unsigned long now=millis();
  imuUpdate();
  if(moving && now-lastCmd>300) applyLR(0,0);            // watchdog
  if(now-lastTel>=100){                                  // telemetría 10 Hz
    lastTel=now; long q[4];
    noInterrupts(); for(int k=0;k<4;k++) q[k]=quadPos[k]; interrupts();
    char b[110];
    if(imuOk)
      snprintf(b,sizeof(b),"T %ld %ld %ld %ld %ld %ld",
        q[0], q[1], q[2], q[3], (long)(yawRad*1000.0f), (long)(gzRad*1000.0f));
    else
      snprintf(b,sizeof(b),"T %ld %ld %ld %ld", q[0], q[1], q[2], q[3]);
    Serial.println(b);
  }
  static unsigned long lastRep=0;                        // salud del I2C cada 10 s
  if(now-lastRep>=10000){ lastRep=now; char r[60];
    snprintf(r,sizeof(r),"R I2C glitches=%lu caidas=%u imu=%s",(unsigned long)imuGlitches,imuCaidas,imuOk?"OK":"NO");
    Serial.println(r); }
  unsigned long per = moving?150:800;                    // LED: rápido si se mueve
  if(now-lastBlink>=per){ lastBlink=now; digitalWrite(13,!digitalRead(13)); }
}
