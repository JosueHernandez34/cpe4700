#ifndef IMU_H
#define IMU_H

int imu_init();
double get_imu_roll();
double get_imu_pitch();
double get_imu_heading();
void imu_cleanup();

#endif