
#ifndef globals.h
  #define globals.h
  #define pinBuzzer 4

  enum { IMU_RUN, IMU_CAL_COM };
    
  struct point_int_t
  {
    int16_t x;
    int16_t y;
    int16_t z;
  };
  typedef struct point_int_t point_int_t;
  
  struct point_long_t
  {
    long x;
    long y;
    long z;
  };
  typedef struct point_long_t point_long_t;
  
  struct point_float_t
  {
    float x;
    float y;
    float z;
  };
  typedef struct point_float_t point_float_t;
  
  struct ypr_t
  {
    float yaw;
    float pitch;
    float roll;
  };
  typedef struct ypr_t ypr_t;


  extern int IMU_Ta_Array[100];  //**OK 
  extern long IMU_Ta_sum;  //**OK
  extern unsigned long IMU_lastIMUupdate;  //**OK
  extern float IMU_yaw;
  extern float IMU_yaw1;  //**OK
  extern int IMU_callCounter;  //**OK
  extern int IMU_errorCounter;  //**OK
  extern boolean IMU_hardwareInitialized;  //**OK
  extern byte IMU_state;  //**OK
  extern unsigned long IMU_lastAHRSTime;  //**OK
  extern unsigned long IMU_now;    //**OK
  extern ypr_t IMU_ypr;  // gyro yaw,pitch,roll    //**OK
  
  // --------- gyro state -----------------------------
  extern point_float_t IMU_gyro;    // gyro sensor data (degree)    //**OK
  extern point_float_t IMU_gyroOfs; // gyro calibration data        //**OK
  extern float IMU_gyroNoise;       // gyro noise                   //**OK
  extern int IMU_gyroCounter;                                       //**OK
  extern boolean IMU_useGyroCalibration; // gyro calibration flag   //**OK
  extern unsigned long IMU_lastGyroTime;                            //**OK

  // --------- acceleration state ---------------------
  extern point_float_t IMU_acc;      // acceleration sensor data     //**OK
  extern point_float_t IMU_accMin;   //**OK
  extern point_float_t IMU_accMax;   //**OK
  extern int IMU_accelCounter;       //**OK
  extern boolean IMU_useAccCalibration;   //**OK
  extern float IMU_accPitch;         //**OK
  extern float IMU_accRoll;          //**OK
  extern point_float_t IMU_accOfs;   //**OK
  extern point_float_t IMU_accScale; //**OK
  extern int IMU_calibAccAxisCounter;  //**OK

  // calibrate acceleration sensor  
  extern boolean IMU_calibrationAvail;  //**OK

  // --------- compass state --------------------------  
  extern point_float_t IMU_com; // compass sensor data (raw)       //**OK
  extern point_float_t IMU_comLast;                                //**OK
  extern point_float_t IMU_comMin; // compass sensor data (raw)    //**OK
  extern point_float_t IMU_comMax; // compass sensor data (raw)    //**OK
  extern point_float_t IMU_comTilt; // compass sensor data (tilt corrected)  //**OK
  extern point_float_t IMU_comOfs;                                 //**OK
  extern point_float_t IMU_comScale;                               //**OK
  extern float IMU_comYaw;         // compass heading (radiant, raw)   //**OK
  extern boolean IMU_useComCalibration;                                //**OK
  
  // calibrate compass sensor  
  extern unsigned long IMU_lastCompassReadTime;                    //**OK
  
  // helpers
  extern boolean IMU_update_enable;                                //**OK
  
  // hardware
  extern boolean IMU_foundNewMinMax;   //**OK
  extern unsigned long IMU_last_time;

#endif

