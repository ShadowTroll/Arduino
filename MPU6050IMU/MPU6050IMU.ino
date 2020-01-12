#define   PIN_INT 12

#include <Wire.h>
#include "MPU6050.h"

MPU6050lib mpu;

float aRes, gRes;               // scale resolutions per LSB for the sensors
int16_t accellast_ms[3];        // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;               // Stores the real accel value in g's
int16_t gyrolast_ms[3];         // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;               // Stores the real gyro value in degrees per seconds
float gyroBias[3] = {0, 0, 0};
float accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
int16_t templast_ms;            // Stores the real internal chip temperature in degrees Celsius
float temperature;
float pitch, yaw, roll;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion

// filter parameters
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

// time variables
uint32_t now_us = 0;
uint32_t last_us = 0;
uint32_t now_ms = 0;
uint32_t last_ms = 0;
float dt = 0.0f;        // integration interval

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(PIN_INT, INPUT);
  digitalWrite(PIN_INT, LOW);

  // initialize MPU6050
  uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);
  if (c == 0x68) {
    mpu.calibrateMPU6050(gyroBias, accelBias);
    mpu.initMPU6050();
  } else {
      Serial.println("Could not connect to MPU6050!");
      while (1);
  }
}

void loop() {
  // time
  now_us = micros();
  now_ms = millis();
  dt = ((now_us - last_us) / 1000000.0f); // sensor time sampling in seconds
  last_us = now_us;
  
  // check sensor for new data
  updateIMU();

  // Madgwick filter update
  updateFilter(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f);

  // data output
  if (now_ms - last_ms > 100) {
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    roll  *= 180.0f / PI;

    Serial.print(yaw, 2);
    Serial.print("\t");
    Serial.print(pitch, 2);
    Serial.print("\t");
    Serial.println(roll, 2);
    
    last_ms = millis();
  }
}


void updateFilter(float ax, float ay, float az, float gx, float gy, float gz) {
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    // Normalize accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
  
    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
    
    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;
    
    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
    
    // Compute and remove gyroscope biases
    gbiasx += gerrx * dt * zeta;
    gbiasy += gerry * dt * zeta;
    gbiasz += gerrz * dt * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;
    
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * dt;
    q2 += (qDot2 -(beta * hatDot2)) * dt;
    q3 += (qDot3 -(beta * hatDot3)) * dt;
    q4 += (qDot4 -(beta * hatDot4)) * dt;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void updateIMU() {
    // check if data ready interrupt (new data available)
  if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) { 
    
    // read current acceleration
    mpu.readAccelData(accellast_ms);
    aRes = mpu.getAres(); // [g/div]

    // convert acceleration
    ax = (float)accellast_ms[0] * aRes;
    ay = (float)accellast_ms[1] * aRes;
    az = (float)accellast_ms[2] * aRes;

    // read current angular velocity
    mpu.readGyroData(gyrolast_ms);
    gRes = mpu.getGres(); // [DPS/div]

    // convert angular velocity
    gx = (float)gyrolast_ms[0] * gRes;
    gy = (float)gyrolast_ms[1] * gRes;
    gz = (float)gyrolast_ms[2] * gRes;

    // read temperature
    templast_ms = mpu.readTempData();

    // convert temperature
    temperature = ((float)templast_ms) / 340. + 36.53;
  }
}
