//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Data structures

// * Please note that in order to increase modularity, global variables were removed from this library.

typedef struct {
    float q0, q1, q2, q3; // Quaternion of sensor frame relative to auxiliary frame
} quaternion_t;

// For every sensor array, their information is organized as: [0]:x_axis, [1]:y_axis, [2]:z_axis
typedef struct {
    quaternion_t orientation; // Quaternion of sensor frame relative to auxiliary frame
    float accel[3]; // Accelerometer measurements
    float gyro[3]; // Gyroscope measurements
    float mag[3]; // Magnetometer measurements
    float beta; // Algorithm gain
    float sample_freq; // Sampling frequency in Hz
} AHRS_data_t;

//---------------------------------------------------------------------------------------------------
// Public function declarations

void MadgwickAHRSinit(AHRS_data_t* imu, float desired_sample_freq);
void MadgwickAHRSupdate(AHRS_data_t *imu);
void MadgwickAHRSupdateIMU(AHRS_data_t *imu);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
