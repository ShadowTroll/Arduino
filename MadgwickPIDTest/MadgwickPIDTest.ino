#include <MadgwickAHRS.h>
//#include <pid.h>
#include <MPU6050.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

Madgwick filter;
//PID pid = PID(50,90,-90,1,0.01,0.01);
MPU6050 accGyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int offax, offay, offaz, offgx, offgy, offgz;
float roll, pitch, yaw;

double accTomPss;

long lastTime, currTime;

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
  #endif
  Serial.begin(19200);
  lastTime = millis();
  accGyro.initialize();
  filter.begin(35);

  Serial.print(accGyro.getFreefallDetectionThreshold()); Serial.println(accGyro.getFreefallDetectionDuration()); 
  Serial.print(accGyro.getMotionDetectionThreshold()); Serial.println(accGyro.getMotionDetectionDuration()); 

    offax = 950;
    offay = 590; 
    offaz = -1100;
    offgx = 670; 
    offgy = -100; 
    offgz = -57;
}

void loop() {
  currTime = millis();
  if (currTime-lastTime >= 35) {
    accGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax += offax;
    ay += offay; 
    az += offaz;
    gx += offgx; 
    gy += offgy; 
    gz += offgz;

    ax = convertRawAcceleration(ax);
    ay = convertRawAcceleration(ay);
    az = convertRawAcceleration(az);
    gx = convertRawGyro(gx);
    gy = convertRawGyro(gy);
    gz = convertRawGyro(gz);
    
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    pitch = filter.getPitch();
    //double pitchDelta = pid.calculate(0, pitch);

    Serial.println(pitch);
    //Serial.print("   Change: "); Serial.println(pitchDelta);   
    
    lastTime = currTime;
  }  
}


float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}
