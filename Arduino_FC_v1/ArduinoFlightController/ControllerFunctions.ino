// Serial debug bools
bool showAccX, showAccY, showAccZ, showGyroX, showGyroY, showGyroZ, showP, showI, showRC, showPWM;

void InitialiseESCS(){
  // Attach each ppm output to their desired pins
  s1.attach(motor1Pin);
  s2.attach(motor2Pin);
  s3.attach(motor3Pin);
  s4.attach(motor4Pin);
  
  
  // Set the PPM value to 0 to initialise the ESCs
  s1.writeMicroseconds(1000);
  s2.writeMicroseconds(1000);
  s3.writeMicroseconds(1000);
  s4.writeMicroseconds(1000);
}

void writePPM(Servo s, float value){
  float v = map(value, 0, 255, 1165, 2000);
  s.writeMicroseconds(v);
}

void InitialiseMPU(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission();

  // Set digital low pass filter to 42 Hz
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(B00001000);
  Wire.endTransmission();
}

void GetMPUValues(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14); // request a total of 14 registers
  int16_t t = Wire.read();
  AcX = -(float)(((t << 8) | Wire.read()) - sOffsetX) / 16384 * 9.81; // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  t = Wire.read();
  AcY = -(float)(((t << 8) | Wire.read()) - sOffsetY) / 16384 * 9.81; // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  t = Wire.read();
  AcZ = (float)(((t << 8) | Wire.read()) - sOffsetZ + 16384) / 16384 * 9.81; // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  t = Wire.read();
  Tmp = (t << 8) | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  t = Wire.read();
  GyX = -(float)(((t << 8) | Wire.read())) * gyroScaleFactor; // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  t = Wire.read();
  GyY = -(float)(((t << 8) | Wire.read())) * gyroScaleFactor; // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  t = Wire.read();
  GyZ = (float)(((t << 8) | Wire.read())) * gyroScaleFactor; // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  t = Wire.read();
}

