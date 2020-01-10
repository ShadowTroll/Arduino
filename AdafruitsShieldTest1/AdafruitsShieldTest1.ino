#include <AFMotor.h>
#define MOTORDEBUG

// Number of steps per output rotation
// Change this as per your motor's specification
const int stepsPerRevolution = 200;

// connect motor to port #2 (M3 and M4)
AF_Stepper motor(stepsPerRevolution, 2);

void setup() {
  Serial.begin(9600);
  Serial.println("Stepper test!");

  motor.setSpeed(40);  // 10 rpm   
}

void loop() {
  /*
  Serial.println("Single coil steps");
  motor.step(100, FORWARD, SINGLE); 
  motor.step(100, BACKWARD, SINGLE); 

  delay(1000);

  Serial.println("Double coil steps");
  motor.step(100, FORWARD, DOUBLE); 
  motor.step(100, BACKWARD, DOUBLE);

  delay(1000);

  Serial.println("Interleave coil steps");
  motor.step(100, FORWARD, INTERLEAVE); 
  motor.step(100, BACKWARD, INTERLEAVE); 

  delay(1000);*/

  Serial.println("Micrsostep steps");
  motor.step(100, FORWARD, MICROSTEP); 
  motor.step(100, BACKWARD, MICROSTEP); 

  delay(1000);
}
