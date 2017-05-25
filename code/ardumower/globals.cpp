
#include "globals.h"




int IMU_Ta_Array[100];
long IMU_Ta_sum;  
unsigned long IMU_lastIMUupdate;
float IMU_yaw;
float IMU_yaw1;
int IMU_callCounter;
int IMU_errorCounter;
boolean IMU_hardwareInitialized;
byte IMU_state;
unsigned long IMU_lastAHRSTime;
unsigned long IMU_now;
ypr_t IMU_ypr;  // gyro yaw,pitch,roll    
point_float_t IMU_gyro;    // gyro sensor data (degree)
point_float_t IMU_gyroOfs; // gyro calibration data      
float IMU_gyroNoise;       // gyro noise                   
int IMU_gyroCounter;
boolean IMU_useGyroCalibration; // gyro calibration flag 
unsigned long IMU_lastGyroTime;             

// --------- acceleration state ---------------------
point_float_t IMU_acc;      // acceleration sensor data     
point_float_t IMU_accGrav;  // acceleration sensor data (gravity corrected)
point_float_t IMU_accMin;   //**OK
point_float_t IMU_accMax;   //**OK
int IMU_accelCounter;       //**OK
boolean IMU_useAccCalibration;   //**OK
float IMU_accPitch;         //**OK
float IMU_accRoll;          //**OK
point_float_t IMU_accOfs;   //**OK
point_float_t IMU_accScale; //**OK
int IMU_calibAccAxisCounter;  //**OK

// calibrate acceleration sensor  
boolean IMU_calibrationAvail;  //**OK

// --------- compass state --------------------------  
point_float_t IMU_com; // compass sensor data (raw)       //**OK
point_float_t IMU_comLast;                                //**OK
point_float_t IMU_comMin; // compass sensor data (raw)    //**OK
point_float_t IMU_comMax; // compass sensor data (raw)    //**OK
point_float_t IMU_comTilt; // compass sensor data (tilt corrected)  //**OK
point_float_t IMU_comOfs;                                 //**OK
point_float_t IMU_comScale;                               //**OK
float IMU_comYaw;         // compass heading (radiant, raw)   //**OK
boolean IMU_useComCalibration;                               //**OK

unsigned long IMU_lastCompassReadTime;                    //**OK

boolean IMU_update_enable;                              //**OK
   
boolean IMU_foundNewMinMax;   //**OK
unsigned long IMU_last_time;


  
