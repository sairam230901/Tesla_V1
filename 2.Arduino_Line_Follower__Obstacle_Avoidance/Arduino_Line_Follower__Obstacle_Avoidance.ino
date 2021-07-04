#include <Servo.h>       //library for servo motors : note that this is pre installed
#include <AFMotor.h>     //library for the motors to control speed and direction of rotation: note that is should be manually installed by us from the manage libraries section. 
#include <NewPing.h>     //library for the ultrasonic sensor: note that is should be manually installed by us from the manage libraries section.
 
//hc-sr04 sensor
#define TRIGGER_PIN A2   //for trigger pin from the ultrasonic sensor to analog input A2
#define ECHO_PIN A3      //for echo pin from ultrasonic sensor to analog input A3
#define max_distance 50  //for obstacle avoidance variable

//ir sensor
#define irLeft A0       //for ir sensor on left to analog input A0
#define irRight A1      //for ir sensor on the right to analog input A1

//motor
#define MAX_SPEED 200   //defining the maximum speed of the motor 
#define MAX_SPEED_OFFSET 20 //the offset variable +/- the actual max_speed of motor

Servo servo;           //creating the obejct of servo

NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance); //the function for ultrasonic sensor for obstacle avoidance

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);  // all defining the PWM of the motors by channel selection from 1-4
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);


int distance = 0;  //initalizing the distance as 0 for starting point.
int leftDistance;  // variable to calculate the left distance from obstacle.
int rightDistance; // variable to calculate the right distance from obstacle
boolean object;    // a boolean variable to detect if an obstacle is present or not
 
void setup() {
  Serial.begin(9600);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  servo.attach(10);
  servo.write(90);

  motor1.setSpeed(120);
  motor2.setSpeed(120);  //setting all motors to speed as mentioned
  motor3.setSpeed(120);
  motor4.setSpeed(120);
}
//the loop function where we check for both obstacles while following the line
void loop() {
  if (digitalRead(irLeft) == 0 && digitalRead(irRight) == 0 ) {
    objectAvoid();
    //forword
  }
  else if (digitalRead(irLeft) == 0 && digitalRead(irRight) == 1 ) {
    objectAvoid();
    Serial.println("TL");
    //leftturn
    moveLeft();
  }
  else if (digitalRead(irLeft) == 1 && digitalRead(irRight) == 0 ) {
    objectAvoid();
    Serial.println("TR");
    //rightturn
    moveRight();
  }
  else if (digitalRead(irLeft) == 1 && digitalRead(irRight) == 1 ) {
    //Stop
    Stop();
  }
}

void objectAvoid() {
  distance = getDistance();
  if (distance <= 15) {
    //stop
    Stop();
    Serial.println("Stop");

    lookLeft();
    lookRight();
    delay(100);
    if (rightDistance <= leftDistance) {
      //left
      object = true;
      turn();
      Serial.println("moveLeft");
    } else {
      //right
      object = false;
      turn();
      Serial.println("moveRight");
    }
    delay(100);
  }
  else {
    //forword
    Serial.println("moveforword");
    moveForward();
  }
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 100;
  }
  return cm;
}

// function for looking left after obstacle detection
int lookLeft () {
  //lock left
  servo.write(150); // setting the angle of servo motor to 150 degrees so as to make the ultrasonic sensor send signals
  delay(500);
  leftDistance = getDistance(); // function call for returning the distance
  delay(100);
  servo.write(90); //setting back the angle to 90 degrees
  Serial.print("Left:");
  Serial.print(leftDistance);
  return leftDistance;
  delay(100);
}

int lookRight() {
  //lock right
  servo.write(30); // setting the angle of servo motor to 30 degrees so as to make the ultrasonic sensor send signals
  delay(500);
  rightDistance = getDistance();  //function call for returning the distance 
  delay(100);
  servo.write(90); //setting back the angle to 90 degrees
  Serial.print("   ");
  Serial.print("Right:");
  Serial.println(rightDistance);
  return rightDistance;
  delay(100);
}

//function to stop the motor
void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

//function to move forward
void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}
//function to move backward
void moveBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

}

//function to turn either right or left :condition dependent
void turn() {
  if (object == false) {
    Serial.println("turn Right");
    moveLeft();
    delay(700);
    moveForward();
    delay(800);
    moveRight();
    delay(900);
    if (digitalRead(irRight) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
  else {
    Serial.println("turn left");
    moveRight();
    delay(700);
    moveForward();
    delay(800);
    moveLeft();
    delay(900);
    if (digitalRead(irLeft) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
}

//function for moving right
void moveRight() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

//function for moving left
void moveLeft() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}
