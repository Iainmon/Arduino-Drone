
#include <Wire.h>

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //declarres variables
float x_rot = 0; //Degrees
float y_rot = 0; //Degrees
float z_rot = 0; //Degrees
double last_time = 0; //Milliseconds
float delta_time = 0;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(byte(0));     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(byte(24));
  Wire.endTransmission(true);
  Serial.begin(115200);
  //calibrate();
}

void loop() {
  
  delta_time = (micros() - last_time) / (float)1000000;
  last_time = micros();  
  x_rot += GyX * delta_time;
  y_rot += GyY * delta_time;
  z_rot += GyZ * delta_time;
 
  populateSensorData();
  valuesToDegrees();
  Serial.print(" | X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);
  
  //debug();
  
}




int intoDeg(int x) {
  x = map(x, -32768, 32767, -2000, 2000);
  return x;
}



void valuesToDegrees() {
  GyX = intoDeg(GyX + 34);
  GyY = intoDeg(GyY + 12);
  GyZ = intoDeg(GyZ + 12);
}

void populateSensorData() {

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

}

void debug() {
  Serial.print(" | X = "); Serial.print(x_rot);
  Serial.print(" | Y = "); Serial.print(y_rot);
  Serial.print(" | Z = "); Serial.println(z_rot);
}

