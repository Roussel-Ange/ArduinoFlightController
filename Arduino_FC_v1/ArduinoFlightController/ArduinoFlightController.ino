//Arduino flight controller
//By gemo
//Using this requires modifying the Servo.h library. The refresh interval should be set at 1000 and not 20000.
#include <Wire.h>
#include <Servo.h>
#include <PPMReader.h>
#include "Quaternion.h"
#include <FastTrig.h>
#include "Position.h"

const float gyroScaleFactor = 1000.0 / 65536; 
const int MPU_addr = 0x68; // I2C address of the MPU-6050
double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

int motor1Pin = 5, motor2Pin = 6, motor3Pin = 9, motor4Pin = 10;
Servo s1, s2, s3, s4;

double p_x = .6, p_y = .6, p_z = .3;
double i_x = .5, i_y = .5, i_z = 0.3;
double d_x = 0.15, d_y =0.15, d_z = 0;

double offsetX = 2.55, offsetY = -2.38, offsetZ = 0.83;
double sOffsetX = -775.8, sOffsetY = -172.80, sOffsetZ = 16374.36;

int rollSensitivity = 30, pitchSensitivity = 30, yawSensitivity = 160; // Sensitivity for each axis on the radio controller

PPMReader rc = PPMReader(7, 7);
void setup() {
  rc.failsafeTimeout = 100000;

  InitialiseMPU();

  InitialiseESCS();

  Serial.begin(115200);
  //Wait for each ESC to be initialised
  delay(3000);

}

int64_t lastMicros; // find dt
double pGX, pGY, pGZ; // proportional action X, Y, Z
double iGX, iGY, iGZ; // integral action X, Y, Z
double angleX, angleY; // angles relative to the ground
double atZ;

bool kill = false;

int32_t watchdogMillis = 0; // receiver signal lost
int32_t watchdogMillis1 = 0; // channel 7 switch
bool arm = false;

double motor1PWM, motor2PWM, motor3PWM, motor4PWM;


Vector3 GlobalVelocity;
Vector3 Position;
double channels[7];
void loop() {
  

  // Get dt from the micros() function
  int16_t deltaMicros = micros() - lastMicros;
  float deltaTime = deltaMicros/1000000.0; // deltatime, in seconds
  if(deltaTime>50.0/1000.0){
    deltaTime = 0;
  }
  lastMicros = micros();

  GetMPUValues();

  Vector3 eulerAngles;
  eulerAngles.z = pGX/180*M_PI;
  eulerAngles.x = pGY/180*M_PI;
  eulerAngles.y = atZ/180*M_PI;
  quaternion q = euler_to_quaternion(eulerAngles); // quaternion to find angle from gyro


  quaternion angular_speed;
  angular_speed.w = 0;
  angular_speed.z = (GyX - offsetX)/180.0*M_PI;
  angular_speed.x = (GyY - offsetY)/180.0*M_PI;
  angular_speed.y = (GyZ - offsetZ)/180.0*M_PI;
  quaternion dot_q;
  dot_q = quaternion_multiply(q, angular_speed);
  quaternion new_q;
  new_q = quaternion_add(q, quaternion_multiply((double)deltaTime * .5, dot_q));
  eulerAngles = quaternion_to_euler(quaternion_normalize(new_q));

  pGX = eulerAngles.z / M_PI * 180.0;
  pGY = eulerAngles.x / M_PI * 180.0;
  atZ = eulerAngles.y / M_PI * 180.0;
  //Calculate the angles from the accelerometer
  angleY = atan2Fast(AcX, AcZ)/M_PI*180.0;
  angleX = atan2Fast(AcY, AcZ)/M_PI*180.0;
  pGX *= (1.0-deltaTime); pGX += deltaTime * angleX;
  pGY *= (1.0-deltaTime); pGY -= deltaTime * angleY;
  
  bool rc_disconnect = false;
  // Get RC channels values
  for(int i = 0; i<7; i++){
    int v = rc.latestValidChannelValue(i+1, 0);
    rc_disconnect = (v <= 500);
    if(v >= 800 && v <= 2100){
      channels[i] = v;
    }
  }
  // map RC channels from 0 to 255
  double roll = (float)-map(channels[3], 1000, 2000, -rollSensitivity * 100, rollSensitivity * 100) / 100;
  double pitch = (float)map(channels[1], 1000, 2000, -pitchSensitivity * 100, pitchSensitivity * 100) / 100;
  double yaw = (float)map(channels[0], 1000, 2000, -yawSensitivity * 100, yawSensitivity * 100) / 100;
  double throttle = (float)map(channels[2], 1000, 2000, 0, 25500) / 100;
  
  
  // Remove before flight
  Debug();

  if(kill == false && arm == true){
    pGZ = GyZ + yaw;

    double RCX = -pGX - pitch;
    double RCY = -pGY - roll;

    iGX += RCX * deltaTime; // integral
    iGY += RCY * deltaTime;
    iGZ += GyZ * deltaTime + yaw * deltaTime;

    double dGX = -GyX - offsetX;
    double dGY = -GyY - offsetY;
    double dGZ = -GyZ - offsetZ;

    motor1PWM = throttle + dGY * d_y + dGX*d_x - dGZ*d_z
      + p_y * RCY + p_x * RCX - p_z * pGZ
      + i_y * iGY + i_x * iGX - i_z * iGZ;

    motor2PWM = throttle + dGY*d_y - dGX*d_x + dGZ*d_z
      + p_y * RCY - p_x * RCX + p_z * pGZ
      + i_y * iGY - i_x * iGX + i_z * iGZ;

     motor3PWM = throttle - dGY*d_y + dGX*d_x + dGZ*d_z
      - p_y * RCY + p_x * RCX + p_z * pGZ
      - i_y * iGY + i_x * iGX + i_z * iGZ;

    motor4PWM = throttle - dGY*d_y - dGX*d_x - dGZ*d_z // D
      - p_y * RCY - p_x * RCX - p_z * pGZ // P
      - i_y * iGY - i_x * iGX - i_z * iGZ; // I

    //write a signal to every esc
    writePPM(s1, constrain(motor1PWM, 0, 255));
    writePPM(s2, constrain(motor2PWM, 0, 255));
    writePPM(s3, constrain(motor3PWM, 0, 255));
    writePPM(s4, constrain(motor4PWM, 0, 255));
  }else{
    writePPM(s1, -30);
    writePPM(s2, -30);
    writePPM(s3, -30);
    writePPM(s4, -30);
  }
  //Failsafe 1: kill switch on channel 7
  if(channels[6] >= 1450){
    kill = true;
    throttle = -2000;
    Serial.println("kill");
  }
  //Failsafe 2: 0.5s watchdog timer for receiver disconnect
  if(!rc_disconnect){
    watchdogMillis = millis();

  }
  else if(millis() - watchdogMillis > 500){
    kill = true;
    //Serial.println("Receiver disconnected.");
  }

  if(channels[4] > 1250){
    watchdogMillis1 = millis();
    arm = true;
  }
  else if(millis() - watchdogMillis1 > 100){
    arm = false;
    pGZ = 0;
    iGX = 0; // integral
    iGY = 0;
    iGZ = 0;
  }
}