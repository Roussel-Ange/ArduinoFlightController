#include <Math.h>
#include <Arduino.h>
#include <FastTrig.h>
struct quaternion{
  double w = 1;
  double x = 0;
  double y = 0;
  double z = 0;
};

struct Vector3{
  double x = 0;
  double y = 0;
  double z = 0;
};
quaternion quaternion_multiply(quaternion q1, quaternion q2){
  quaternion q;
  q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
  q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
  q.y = q1.w*q2.y + q1.x*q2.z + q1.y*q2.w - q1.z*q2.x;
  q.z = q1.w*q2.z - q1.x*q2.y + q1.y*q2.x + q1.z*q2.w; 
  return q;
};

quaternion quaternion_multiply(double d, quaternion _q){
  quaternion q;
  q.w = d*_q.w;
  q.x = d*_q.x;
  q.y = d*_q.y;
  q.z = d*_q.z;
  return q;
};

quaternion quaternion_add(quaternion q1, quaternion q2){
  quaternion q;
  q.w = q1.w + q2.w;
  q.x = q1.x + q2.x;
  q.y = q1.y + q2.y;
  q.z = q1.z + q2.z;
  return q;
};

quaternion quaternion_divide(quaternion _q, double d){
  quaternion q;
  q.w = _q.w/d;
  q.x = _q.x/d;
  q.y = _q.y/d;
  q.z = _q.z/d;
  return q;
};

quaternion quaternion_normalize(quaternion _q){
  quaternion q;
  double norm;
  norm = sqrt(_q.w * _q.w + _q.x * _q.x + _q.y * _q.y + _q.z * _q.z);
  q = quaternion_divide(_q, norm);
  return q;
};

Vector3 quaternion_to_euler(quaternion q) {
  Vector3 eulerAngles;
  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  double roll;
  roll = atan2(sinr_cosp, cosr_cosp);
  eulerAngles.x = roll;

  // Pitch (y-axis rotation)
  double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  double pitch;
  if (fabs(sinp) >= 1.0)
    pitch = copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
  else
    pitch = sin(sinp);
  eulerAngles.y = pitch;

  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw;
  yaw = atan2(siny_cosp, cosy_cosp);
  eulerAngles.z = yaw;

  return eulerAngles;  
};

quaternion euler_to_quaternion(Vector3 euler) {
  Vector3 _euler = euler;

  // Calculate the half angles
  float c1 = cos(_euler.z / 2);
  float s1 = sin(_euler.z / 2);
  float c2 = cos(_euler.y / 2);
  float s2 = sin(_euler.y / 2);
  float c3 = cos(_euler.x / 2);
  float s3 = sin(_euler.x / 2);

  // Compute the quaternion
  quaternion q;
  q.w = c1 * c2 * c3 + s1 * s2 * s3;
  q.x = s3 * c2 * c1 - c3 * s2 * s1;
  q.y = c3 * s2 * c1 + s3 * c2 * s1;
  q.z = c3 * c2 * s1 - s3 * s2 * c1;

  // Normalize the quaternion (optional)
  q = quaternion_normalize(q);

  return q;
};