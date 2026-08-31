// Prueba mínima Due: LED L (pin 13) parpadea 5 Hz + contador por Serial (115200)
unsigned long n=0;
void setup(){ pinMode(13,OUTPUT); Serial.begin(115200); }
void loop(){
  digitalWrite(13,HIGH); delay(100); digitalWrite(13,LOW); delay(100);
  Serial.print("vivo "); Serial.println(n++);
}
