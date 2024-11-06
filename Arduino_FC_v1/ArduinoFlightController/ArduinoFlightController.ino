//Arduino flight controller
//By gemo
//Using this requires modifying the Servo.h library. The minimum ppm frame time should be set at 1000 and not 20000.
#include <Wire.h>
#include <Servo.h>
#include <PPMReader.h>
const float gyroScaleFactor = 1000.0 / 65536; 
const int MPU_addr = 0x68; // I2C address of the MPU-6050
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

int motor1Pin = 5, motor2Pin = 6, motor3Pin = 9, motor4Pin = 10;
Servo s1, s2, s3, s4;
int gimbalPitchPin = 15, gimbalRollPin = 14;
Servo gimbalPitchServo, gimbalRollServo;


double p_x =  .37, p_y = .42, p_z = 1.5;
double i_x = .5, i_y = .5, i_z = 0;
double d_x = 0.14, d_y =0.15, d_z = -0.3;


double offsetX = -175.00, offsetY = 131.85, offsetZ = 58.65;
double sOffsetX = -81.99, sOffsetY = 336.95, sOffsetZ = 15986.71;

double angleX, angleY;

int rollSensitivity = 22.5, pitchSensitivity = 22.5, yawSensitivity = 80; // Sensitivity for each axis on the radio controller

PPMReader rc = PPMReader(7, 7);
void setup() {
  gimbalPitchServo.attach(gimbalPitchPin);
  gimbalRollServo.attach(gimbalRollPin);

  InitialiseMPU();

  InitialiseESCS();

  Serial.begin(115200);
  //Wait for each ESC to be initialised
  delay(10000);

}

int64_t lastMicros = 0;
double lastRoll, lastPitch, lastYaw;

float pGX, pGY, pGZ;
double gPGX, gPGY;
float iGX, iGY, iGZ;

double channels[7];

// Serial debug bools
bool showAccX, showAccY, showAccZ, showGyroX, showGyroY, showGyroZ, showP, showI, showRC, showPWM;
bool kill = false;

int32_t watchdogMillis = 0; // receiver signal lost
int32_t watchdogMillis1 = 0; // channel 7 switch
bool arm = false;

int derivativeIterations;

double motor1PWM, motor2PWM, motor3PWM, motor4PWM;

double pitchAverage, rollAverage;
double pitchDerivative, rollDerivative;
void loop() {
  delay(1);

  GetMPUValues();
  
  // Get dt from the micros() function
  float deltaTime = (float)(micros() - lastMicros) / 1000000;


  if(deltaTime>50.0/1000.0){
    deltaTime = 0;
  }

  pGX -= GyX * deltaTime; 
  pGY -= GyY * deltaTime;

  //Calculate the angles from the accelerometer
  angleY = atan2(AcX, AcZ)/PI*180;
  angleX = atan2(AcY, AcZ)/PI*180;

  pGX = (1-deltaTime) * pGX - deltaTime * angleX;
  pGY = (1-deltaTime) * pGY + deltaTime * angleY;
  lastMicros = micros();
  UpdateGimbal();

  // Get RC channels values
  for(int i = 0; i<7; i++){
    channels[i] = rc.latestValidChannelValue(i+1, 0);
  }


  
  double roll = (float)-map(channels[3], 1000, 2000, -rollSensitivity * 100, rollSensitivity * 100) / 100;
  double pitch = (float)map(channels[1], 1000, 2000, -pitchSensitivity * 100, pitchSensitivity * 100) / 100;
  double yaw = (float)map(channels[0], 1000, 2000, -yawSensitivity * 100, yawSensitivity * 100) / 100;
  double throttle = (float)map(channels[2], 1000, 2000, 0, 25500) / 100;
  
  
  // Remove before flight
  Debug();

  if(kill == false && arm == true){


    pGZ += GyZ * deltaTime + yaw * deltaTime;



    double RCX = pGX - pitch;
    double RCY = pGY - roll;

    iGX += RCX * deltaTime; // integral
    iGY += RCY * deltaTime;
    iGZ += pGZ * deltaTime;


    double dGX = -GyX;
    double dGY = -GyY;
    double dGZ = -GyZ;


    motor1PWM = throttle + -dGY * d_y - dGX*d_x - dGZ*d_z
      - p_y * RCY - p_x * RCX - p_z * pGZ
      - i_y * iGY - i_x * iGX - i_z * iGZ;

    motor2PWM = throttle + -dGY*d_y + dGX*d_x + dGZ*d_z
      - p_y * RCY + p_x * RCX + p_z * pGZ
      - i_y * iGY + i_x * iGX + i_z * iGZ;

    motor3PWM = throttle + dGY*d_y - dGX*d_x + dGZ*d_z
    + p_y * RCY - p_x * RCX + p_z * pGZ
    + i_y * iGY - i_x * iGX + i_z * iGZ;

    motor4PWM = throttle + dGY*d_y + dGX*d_x - dGZ*d_z // D
    + p_y * RCY + p_x * RCX - p_z * pGZ // P
    + i_y * iGY + i_x * iGX - i_z * iGZ; // I


    writePPM(s1, constrain(motor1PWM, 0, 255));
    writePPM(s2, constrain(motor2PWM, 0, 255));
    writePPM(s3, constrain(motor3PWM, 0, 255));
    writePPM(s4, constrain(motor4PWM, 0, 255));
  }else{
    writePPM(s1, -30);
    writePPM(s2, -30);
    writePPM(s3, -30);
    writePPM(s4, -30);
    Serial.println("disarmed");
  }
  //Failsafe 1: kill switch on channel 7
  if(channels[6] >= 1450){
    kill = true;
    throttle = -2000;
    Serial.println("kill");
  }
  //Failsafe 2: 0.5s watchdog timer for receiver disconnect
  if(channels[0] > 500){
    watchdogMillis = millis();

  }
  else if(millis() - watchdogMillis > 500){
    kill = true;
  }

  if(channels[4] > 1250){
    watchdogMillis1 = millis();
    arm = true;
  }
  else if(millis() - watchdogMillis1 > 100){
    arm = false;
    lastMicros = micros();
    pGZ = 0;


    iGX = 0; // integral
    iGY = 0;
    iGZ = 0;
  }
}