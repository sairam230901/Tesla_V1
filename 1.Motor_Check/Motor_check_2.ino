#include<AFMotor.h>
#define MAX_SPEED 200   //defining the maximum speed of the motor 
#define MAX_SPEED_OFFSET 20 //the offset variable +/- the actual max_speed of motor
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);  // all defining the PWM of the motors by channel selection from 1-4
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);
void setup() {
  // put your setup code here, to run once:
  motor1.setSpeed(120);
  motor2.setSpeed(120);  //setting all motors to speed as mentioned
  motor3.setSpeed(120);
  motor4.setSpeed(120);

}

void loop() {
  // put your main code here, to run repeatedly:
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);

}
