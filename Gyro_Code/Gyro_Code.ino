
#include <Wire.h>

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //declarres variables
float x_off, y_off, z_off;
float x_rot = 0; //Degrees
float y_rot = 0; //Degrees
float z_rot = 0; //Degrees
double last_time = 0; //Milliseconds
float delta_time = 0;

int print_loops = 0;

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
  calibrate();
}

void loop() {

  add_values_to_sum();
  populateSensorData();
  valuesToDegrees(false);
  debugRot(); 
  //debugVel();
  
}



void calibrate(){
  double xavg = 0;
  double yavg = 0;
  double zavg = 0;
  int tries = 100;
  for(int i = 0; i < tries; i++){
    populateSensorData();
    valuesToDegrees(true);
    xavg += GyX;
    yavg += GyY;
    zavg += GyZ;
  }
  x_off = xavg / tries;
  y_off = yavg / tries;
  z_off = zavg / tries;
}



void add_values_to_sum(){
  delta_time = (micros() - last_time) / (float)1000000;
  last_time = micros();
  x_rot = circle_clamp(x_rot + GyX * delta_time);
  y_rot = circle_clamp(x_rot + GyY * delta_time);
  z_rot = circle_clamp(x_rot + GyZ * delta_time);
  
}

float circle_clamp(float input){
  if(input > 180){
    input = -180 + (input - 180);
  }
  else if(input < -180){
    input = 180 - (input + 180);
  }
}

float intoDeg(int x) {
  return custom_map(x, -32768, 32767, -2000, 2000);
}

float custom_map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return ((float)((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
}

void valuesToDegrees(bool cal) {
  if(cal == false){
    GyX = intoDeg(GyX) - x_off;
    GyY = intoDeg(GyY) - y_off;
    GyZ = intoDeg(GyZ) - z_off;
  }
  else{
    GyX = intoDeg(GyX);
    GyY = intoDeg(GyY);
    GyZ = intoDeg(GyZ);
  }
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

void debugRot() {
  if(print_loops > 100){
    print_loops = 0;
    Serial.print(" | Xrot = "); Serial.print(x_rot);
    Serial.print(" | Yrot = "); Serial.print(y_rot);
    Serial.print(" | Zrot = "); Serial.println(z_rot);
  }
  else{
    print_loops += 1;
  }
}

void debugVel() {
  Serial.print(" | X = "); Serial.print(GyX);
  Serial.print(" | Y = "); Serial.print(GyY);
  Serial.print(" | Z = "); Serial.println(GyZ);
}

