#include <Wire.h>

const String SEPARATOR = ",\t";
const float ACC_COEF = 9.8155 / 16384.0;
const float ALPHA = 0.001;

int Accel_output[3];
int Gyro_output[3];

void writeTo(uint8_t devAddr, uint8_t toAddress, uint8_t val) {
  Wire.beginTransmission(devAddr);
  Wire.write(toAddress);
  Wire.write(val);
  Wire.endTransmission();
}

// Adapted from Arduino.cc/forums for reading in sensor data
void readFrom(uint8_t devAddr, uint8_t fromAddress, int num, uint8_t result[]) {
  Wire.beginTransmission(devAddr);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)devAddr, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.read();
    i++;
  }
}

void writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
  uint8_t b;
  readFrom(devAddr, regAddr, 1, &b);
  b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
  writeTo(devAddr, regAddr, b);
}

void getGyroscopeReadings(int Gyro_output[]) {
  uint8_t buffer[6];
  readFrom(0x68, 0x43, 6, buffer);
  Gyro_output[0] = (((int)buffer[0]) << 8 ) | buffer[1];
  Gyro_output[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  Gyro_output[2] = (((int)buffer[4]) << 8 ) | buffer[5];
}

void getAccelerometerReadings(int Accel_output[]) {
  uint8_t buffer[6];
  readFrom(0x68, 0x3B, 6, buffer);
  Accel_output[0] = (((int)buffer[0]) << 8 ) | buffer[1];
  Accel_output[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  Accel_output[2] = (((int)buffer[4]) << 8 ) | buffer[5];
}

float getTemperature() {
  uint8_t buffer[2];
  readFrom(0x68, 0x41, 2, buffer);
  int result = (((int)buffer[0]) << 8 ) | buffer[1];
  return ((float) result) / 340 + 36.53;
}

float lerp(float a, float b, float alpha) { 
  return a + alpha * (b - a);
}

float accPrevX, accPrevY, accPrevZ, tempPrev;

void setup() {
  delay(5);
  Wire.begin();
  Serial.begin(115200); // Registers set

  // Disable sleep mode:
  writeBit(0x68, 0x6B, 6, 0);

  getAccelerometerReadings(Accel_output);
  accPrevX = Accel_output[0] * ACC_COEF;
  accPrevY = Accel_output[1] * ACC_COEF;
  accPrevZ = Accel_output[2] * ACC_COEF;
  tempPrev = getTemperature();
}

void loop() {
  getGyroscopeReadings(Gyro_output);
  getAccelerometerReadings(Accel_output);
  long t = millis();
  float accX = lerp(accPrevX, Accel_output[0] * ACC_COEF, ALPHA);
  float accY = lerp(accPrevY, Accel_output[1] * ACC_COEF, ALPHA);
  float accZ = lerp(accPrevZ, Accel_output[2] * ACC_COEF, ALPHA);

  float gravity = sqrt(accX * accX + accY * accY + accZ * accZ);
  
  float temp = lerp(tempPrev, getTemperature(), ALPHA);

  Serial.print(accX);
  Serial.print(SEPARATOR);
  Serial.print(accY);
  Serial.print(SEPARATOR);
  Serial.print(accZ);
  Serial.print(SEPARATOR);
  Serial.print(gravity);
  Serial.print(SEPARATOR);
  Serial.print(temp);
  Serial.print(SEPARATOR);
  accPrevX = accX;
  accPrevY = accY;
  accPrevZ = accZ;
  tempPrev = temp;
  
//  Serial.print(Gyro_output[0]);
//  Serial.print(SEPARATOR);
//  Serial.print(Gyro_output[1]);
//  Serial.print(SEPARATOR);
//  Serial.print(Gyro_output[2]);
//  Serial.print(SEPARATOR);
  Serial.print(t);
  Serial.println();
}

