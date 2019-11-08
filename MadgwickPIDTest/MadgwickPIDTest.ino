#include <MadgwickAHRS.h>
#include <MPU6050.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

Madgwick filter;
MPU6050 accGyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int offax, offay, offaz, offgx, offgy, offgz;
float roll, pitch, yaw;

long lastTime, currTime;

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
  #endif
  Serial.begin(19200);
  lastTime = micros();
  filter.begin(200);
  accGyro.initialize();
  accGyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  accGyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  offax = 950;
  offay = 590; 
  offaz = -1100;
  offgx = 670; 
  offgy = -100; 
  offgz = -57;
}

void loop() {
  currTime = micros();
  if (currTime - lastTime >= 1000) {
    accGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    ax += offax;
    ay += offay; 
    az += offaz;
    gx += offgx; 
    gy += offgy; 
    gz += offgz;

    float ax_f = convertRawAcceleration(ax);
    float ay_f = convertRawAcceleration(ay);
    float az_f = convertRawAcceleration(az);
    float gx_f = convertRawGyro(gx);
    float gy_f = convertRawGyro(gy);
    float gz_f = convertRawGyro(gz);
    
    filter.updateIMU(gx_f, gy_f, gz_f, ax_f, ay_f, az_f);
    
    pitch = filter.getPitch();
    roll = filter.getRoll();
   
    Serial.println(roll);
    /*Serial.print(gx_f);
    Serial.print("\t");
    Serial.print(gy_f);
    Serial.print("\t");
    Serial.println(gz_f);*/
    
    lastTime = currTime;
  }  
}


float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = ((float)gRaw * 500.0) / 32768.0;
  return g;
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = ((float)aRaw * 2.0) / 32768.0;
  return a;
}
