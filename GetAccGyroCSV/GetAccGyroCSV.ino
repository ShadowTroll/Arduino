#include <MPU6050.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accGyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

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
  Serial.println("ax,  ay,  az,  gx,  gy,  gz");
}

void loop() {
  currTime = millis();
  if (currTime-lastTime >= 35) {
    accGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    /*ax += -3003;
    ay += 1069; 
    az += 927;
    gx += 164; 
    gy += -27; 
    gz += -18;*/
    
    Serial.print(ax+=3003);Serial.print(",");Serial.print(ay+=-1069);Serial.print(",");Serial.print(az+=-927);Serial.print(",");
    Serial.print(gx+=-164);Serial.print(",");Serial.print(gy+=27);Serial.print(",");Serial.println(gz+=18);
    
    lastTime = currTime;
  }  
}
