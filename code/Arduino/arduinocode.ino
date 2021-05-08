//                  ID SPECIFICATIONS                  //
//                                                      
// 0xF0 -> leitura setpoint     // 0xE0 -> escrita setpoint
// 0xF1 -> leitura periodo
// 0xF2 -> leitura pwm          // 0XE2 -> escrita pwm
// 0xF3 -> leitura kp           // 0xE3 -> escrita kp
// 0xF4 -> leitura ki           // 0xE4 -> escrita ki
// 0xF5 -> leitura kd           // 0xE5 -> escrita kd
// 0xAA -> Set Malha aberta 0        // 0xFF -> Set Malha fechada 1
// 0xFA -> Lê malha, 0 aberta 1 fechada
#include <Wire.h>

#define ADRESS 0x58 >> 1
#define FRAME_BUFFER_BYTE 2

//motor constants
const float reduction = 50;
//encoder constants
const float rising = 6;
//Convert period to speed in radians, period is in us
const double to_rad_s = (((2*3.141592)/reduction)/rising)*1000000;

void setup() {
  Wire.begin();
  Wire.setClock(100000);
  Serial.begin(57600);
  setClosedLoop();
  setSetpoint(2);
  setKP(10);
  setKI(1);
  setKD(0);
  
}

void loop() {
  Serial.print((to_rad_s/getPeriod()), DEC);
  Serial.print(",");
  Serial.println(getPWM(), DEC);
  delay(16.66);
}

int bitwiseFrame(byte * bufferFrame) {
  int value = bufferFrame[0];
  value = value << 8;
  value |= bufferFrame[1];
  return value;
}

byte * readFrame(){
  static byte bufferFrame[FRAME_BUFFER_BYTE];
  byte i = 0;
  Wire.requestFrom(ADRESS, FRAME_BUFFER_BYTE);
  while (Wire.available()) {
    bufferFrame[i] = Wire.read();
    i++;
  }
  
  return bufferFrame;
}

void writeFrame(byte id, int value) {
  byte byte_0 = (value >> 8) & 0xFF;
  byte byte_1 = value & 0xFF;
  
  Wire.beginTransmission(ADRESS);
  Wire.write(id);
  Wire.write(byte_0);
  Wire.write(byte_1);
  Wire.endTransmission();
}

int getSetpoint() {
  writeFrame(0xF0, 0);
  byte * bufferFrame = readFrame();
  return bitwiseFrame(bufferFrame);
}

int getKD() {
  writeFrame(0xF5, 0);
  byte * bufferFrame = readFrame();
  return bitwiseFrame(bufferFrame);
}

int getKI() {
  writeFrame(0xF4, 0);
  byte * bufferFrame = readFrame();
  return bitwiseFrame(bufferFrame);
}

int getKP() {
  writeFrame(0xF3, 0);
  byte * bufferFrame = readFrame();
  return bitwiseFrame(bufferFrame);
}

int getPWM() {
  writeFrame(0xF2, 0);
  byte * bufferFrame = readFrame();
  return bufferFrame[0];
}

int getPeriod() {
  writeFrame(0xF1, 0);
  byte * bufferFrame = readFrame();
  return bitwiseFrame(bufferFrame) / 2;
}

int getLoop() {
  writeFrame(0xFA, 0);
  byte * bufferFrame = readFrame();
  return bufferFrame[0];
}

void setSetpoint(int value) {
  writeFrame(0xE0, value);
}

void setKD(int value) {
  writeFrame(0xE5, value);
}

void setKI(int value) {
  writeFrame(0xE4, value);
}

void setKP(int value) {
  writeFrame(0xE3, value);
}

// mudar método abaixo para aceitar o frame default
void setPWM(byte value) {
  //writeFrame(0xE3, value);
  
  Wire.beginTransmission(ADRESS);
  Wire.write(0xE2);
  Wire.write(0);
  Wire.write(value);
  Wire.endTransmission();
}

void setOpenLoop(){
  writeFrame(0xAA, 0);
}

void setClosedLoop(){
  writeFrame(0xFF, 0);
}

void step(){
  double time = millis();
  setPWM(170);
  while((millis() - time) < 5000){
    Serial.print(to_rad_s/getPeriod(), DEC);
    Serial.println(",");
    delay(16.66);
  }
  //170 + 255*0.2
  setPWM(221);
}
