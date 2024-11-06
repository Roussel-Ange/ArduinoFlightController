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

void UpdateGimbal(){
  gimbalPitchServo.write(pGX+115);
  gimbalRollServo.write(pGY+110);
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
  Wire.write(0x03);
  Wire.endTransmission();

    // Set digital low pass filter to 42 Hz
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
  AcX = (float)(((t << 8) | Wire.read()) - sOffsetX) / 16384 * 9.81; // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  t = Wire.read();
  AcY = (float)(((t << 8) | Wire.read()) - sOffsetY) / 16384 * 9.81; // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  t = Wire.read();
  AcZ = (float)(((t << 8) | Wire.read()) - sOffsetZ + 16384) / 16384 * 9.81; // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  t = Wire.read();
  Tmp = (t << 8) | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  t = Wire.read();
  GyX = (float)(((t << 8) | Wire.read()) - offsetX) * gyroScaleFactor; // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  t = Wire.read();
  GyY = (float)(((t << 8) | Wire.read()) - offsetY) * gyroScaleFactor; // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  t = Wire.read();
  GyZ = (float)(((t << 8) | Wire.read()) - offsetZ  ) * gyroScaleFactor; // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  t = Wire.read();
}

void Debug(){
  if(Serial.available() > 0){
        String message = Serial.readString();
    message.trim();
    Serial.println(message);
    if(message == "Accelerometer X"){showAccX = !showAccX;}
    if(message == "Accelerometer Y"){showAccY = !showAccY;}
    if(message == "Accelerometer Z"){showAccZ = !showAccZ;}
    if(message == "Gyro X"){showGyroX = !showGyroX;}
    if(message == "Gyro Y"){showGyroY = !showGyroY;}
    if(message == "Gyro Z"){showGyroZ = !showGyroZ;}
    if(message == "Prop"){showP = !showP;}
    if(message == "Integral"){showI = !showI;}
    if(message == "RC"){showRC = !showRC;}
    if(message == "Kill"){kill = !kill;}
    if(message == "PWM"){showPWM = !showPWM;}
    if(message == "Calibrate"){calibrate();}
  }
  
  if(showRC){
      for(int i = 0; i<7; i++){
        Serial.print(channels[i]); Serial.print(" // ");
      }
      Serial.println();
  }
  // Accelerometer
  if(showAccX){
    Serial.print("AccX: "); Serial.println(AcX);
  }
  if(showAccY){
    Serial.print("AccY: "); Serial.println(AcY);
  }
  if(showAccZ){
    Serial.print("AccZ: "); Serial.println(AcZ);
  }

  //Gyroscope
  if(showGyroX){
    Serial.print("GX: "); Serial.println(GyY);
  }
  if(showGyroY){
    Serial.print("GY: "); Serial.println(GyY);
  }
  if(showGyroZ){
    Serial.print("GZ: "); Serial.println(GyZ);
  }
  if(showP){
    Serial.print("gPGX: "); Serial.print(pGX); Serial.print(" // ");
    Serial.print("gPGY: "); Serial.print(pGY); Serial.print(" // ");
    Serial.print("pGZ: "); Serial.println(pGZ);
  }

  if(showI){
    Serial.print("iGX: "); Serial.print(iGX); Serial.print(" // ");
    Serial.print("iGY: "); Serial.print(iGY); Serial.print(" // ");
    Serial.print("iGZ: "); Serial.println(iGZ);
  }
  
  if(showPWM){
    Serial.print("1: "); Serial.print(motor1PWM); Serial.print(" // ");
    Serial.print("2: "); Serial.print(motor2PWM); Serial.print(" // ");
    Serial.print("2: "); Serial.print(motor3PWM); Serial.print(" // ");
    Serial.print("4: "); Serial.println(motor4PWM);
  }
}

void writePPM(Servo s, float value){
  float v = map(value, 0, 255, 1085, 2000);
  s.writeMicroseconds(v);
}

void calibrate(){

  for(int i = 0; i < 5000; i++){
      Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14); // request a total of 14 registers
    int16_t t = Wire.read();
    AcX = (t << 8) | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    t = Wire.read();
    AcY = (t << 8) | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    t = Wire.read();
    AcZ = (t << 8) | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    t = Wire.read();
    Tmp = (t << 8) | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    t = Wire.read();
    GyX = (float)(((t << 8) | Wire.read())); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    t = Wire.read();
    GyY = (float)(((t << 8) | Wire.read())); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    t = Wire.read();
    GyZ = (float)(((t << 8) | Wire.read()) ); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    t = Wire.read();

    offsetX += GyX;
    offsetY += GyY;
    offsetZ += GyZ;
    sOffsetX += AcX;
    sOffsetY += AcY;
    sOffsetZ += AcZ;
    delay(1);
  }
  offsetX /= 5000;
  offsetY /= 5000;
  offsetZ /= 5000;
  sOffsetX /= 5000;
  sOffsetY /= 5000;
  sOffsetZ /= 5000;
  
  Serial.println(offsetX);
  Serial.println(offsetY);
  Serial.println(offsetZ);
  Serial.println(sOffsetX);
  Serial.println(sOffsetY);
  Serial.println(sOffsetZ);  

}