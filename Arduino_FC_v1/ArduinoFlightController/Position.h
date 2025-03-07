#include <math.h>

#define G 9.81        // Gravity in m/s²

// Function to apply rotation matrix and transform local accelerations to global frame
Vector3 transformToGlobal(Vector3 accel, double roll, double pitch, double yaw) {
    Vector3 globalAccel;

    // Convert angles from degrees to radians
    double phi = roll*180/M_PI;
    double theta = pitch*180/M_PI;
    double psi = yaw*180/M_PI;

    // Compute trigonometric values
    double c_phi = icos(phi), s_phi = isin(phi);
    double c_theta = icos(theta), s_theta = isin(theta);
    double c_psi = icos(psi), s_psi = isin(psi);
    // Rotation matrix multiplication
    globalAccel.x = (c_psi * c_theta) * accel.x +
                    (c_psi * s_theta * s_phi - s_psi * c_phi) * accel.y +
                    (c_psi * s_theta * c_phi + s_psi * s_phi) * accel.z;

    globalAccel.y = (s_psi * c_theta) * accel.x +
                    (s_psi * s_theta * s_phi + c_psi * c_phi) * accel.y +
                    (s_psi * s_theta * c_phi - c_psi * s_phi) * accel.z;

    globalAccel.z = (-s_theta) * accel.x +
                    (c_theta * s_phi) * accel.y +
                    (c_theta * c_phi) * accel.z;
    return globalAccel;
}

// Function to update velocity and position
void updatePosition(Vector3 *position, Vector3 *velocity, Vector3 accelGlobal, double deltaTime) {
    // Integrate acceleration to update velocity
    velocity->x += accelGlobal.x * deltaTime;
    velocity->y += accelGlobal.y * deltaTime;
    velocity->z += (accelGlobal.z - G) * deltaTime;  // Gravity compensation

    // Integrate velocity to update position
    position->x += velocity->x * deltaTime;
    position->y += velocity->y * deltaTime;
    position->z += velocity->z * deltaTime;
}