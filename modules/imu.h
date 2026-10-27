#ifndef IMU_H
#define IMU_H

int imu_init();
double get_imu_roll();
double get_imu_pitch();
double get_imu_heading();
void imu_cleanup();

/* ---- Added for Lab 05 (new functions only) ----
 * Gyro angular rate about the sensor axes, degrees per second.
 * Returns 0 on success, -1 on a failed read (the outputs are then left unchanged). */
int get_imu_gyro(double *gx, double *gy, double *gz);

/* Pitch rate in the SAME convention as the corrected pitch (0 upright, forward lean positive),
 * degrees per second, from the gyro. Which gyro axis is pitch and its sign are NOT SET until the
 * pitch_probe tool has measured them on the real robot: until then this returns NAN, never a
 * made-up number. Set IMU_PITCH_RATE_AXIS (0=X, 1=Y, 2=Z) and IMU_PITCH_RATE_SIGN (+1 or -1)
 * below once measured. */
#define IMU_PITCH_RATE_AXIS  -1     /* NOT SET */
#define IMU_PITCH_RATE_SIGN   0     /* NOT SET */
double get_imu_pitch_rate(void);

#endif