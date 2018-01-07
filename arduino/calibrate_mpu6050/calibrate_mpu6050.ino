#include <Wire.h>

const String SEPARATOR = "\t";
const float GRAVITY = 9.8155;
const float ACC_COEF = 1.0 / 16384.0;
const float ALPHA = 0.05;

int accOutput[3];

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

void getAccelerometerReadings(int accOutput[]) {
  uint8_t buffer[6];
  readFrom(0x68, 0x3B, 6, buffer);
  accOutput[0] = (((int)buffer[0]) << 8 ) | buffer[1];
  accOutput[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  accOutput[2] = (((int)buffer[4]) << 8 ) | buffer[5];
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

  getAccelerometerReadings(accOutput);
  accPrevX = accOutput[0] * ACC_COEF;
  accPrevY = accOutput[1] * ACC_COEF;
  accPrevZ = accOutput[2] * ACC_COEF;
  tempPrev = getTemperature();
}

void loop() {
  getAccelerometerReadings(accOutput);
  long t = millis();
  float accX = lerp(accPrevX, accOutput[0] * ACC_COEF, ALPHA);
  float accY = lerp(accPrevY, accOutput[1] * ACC_COEF, ALPHA);
  float accZ = lerp(accPrevZ, accOutput[2] * ACC_COEF, ALPHA);

  float temp = lerp(tempPrev, getTemperature(), ALPHA);

  float f11 =  0.9957;  float f12 = 0.0000;  float f13 = 0.0000;
  float f21 =  0.0004;  float f22 = 0.9912;  float f23 = 0.0000;
  float f31 = -0.0018;  float f32 = 0.0028;  float f33 = 0.9795;

  float bX = 0.0294;
  float bY = -0.0015;
  float bZ = -0.0768;

  float tX = accX - bX;
  float tY = accY - bY;
  float tZ = accZ - bZ;
  float calX = f11 * tX + f12 * tY + f13 * tZ;
  float calY = f21 * tX + f22 * tY + f23 * tZ;
  float calZ = f31 * tX + f32 * tY + f33 * tZ;

  Serial.print("Raw: (");
  Serial.print(accX);
  Serial.print(", ");
  Serial.print(accY);
  Serial.print(", ");
  Serial.print(accZ);
  Serial.print(")  |g|=");
  Serial.print(sqrt(accX * accX + accY * accY + accZ * accZ));
  Serial.print(SEPARATOR);
  Serial.print(SEPARATOR);
  Serial.print("Calibrated: (");
  Serial.print(calX);
  Serial.print(", ");
  Serial.print(calY);
  Serial.print(", ");
  Serial.print(calZ);
  Serial.print(")  |g|=");
  Serial.print(sqrt(calX * calX + calY * calY + calZ * calZ));
  Serial.print(SEPARATOR);
  Serial.print(SEPARATOR);
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print(SEPARATOR);
  Serial.print(SEPARATOR);
  Serial.print("Time: ");
  Serial.print(t / 1000.0);
  Serial.println();

  accPrevX = accX;
  accPrevY = accY;
  accPrevZ = accZ;
  tempPrev = temp;
}

