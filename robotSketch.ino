/*EENG 2301 Final Project
The University of Texas at Tyler

This file enables the robot detailed on the report with avoid obstacles using echolocation*/
#include<Servo.h>
#include<NewPing.h>

//motor driver constants
#define INPUT1_PIN 2
#define INPUT2_PIN 4
#define INPUT3_PIN 7
#define INPUT4_PIN 8

#define ENABLE_A 3 //pwm (pulse width modulation pins): this can alter the speed of the motor
#define ENABLE_B 11 //pwm

//servo motor constants
#define SRVO_PIN 13
#define RIGHT_ANGLE 60
#define LEFT_ANGLE 150

//ultra-sonic sensor constants
#define ULTR_ECHO_PIN 5
#define ULR_TRIG_PIN 6
#define MAX_DIST 400 //maximum distance measured by the ultrasonic sound sensor

//minimum safe distance in cm
#define TIGGER_DIST 15 
//
NewPing sonar(ULR_TRIG_PIN, ULTR_ECHO_PIN, MAX_DIST);
Servo servoMotor;

int distance[] = {0,0,0}; //getting three distances

void setup(){
  setupMotorPins();
  servoMotor.attach(SRVO_PIN);
  }

void loop(){
  moveForward(500); // move for 300 ms and stop

  //look left and get distance
  servoMotor.write(LEFT_ANGLE);
  delay(300);
  distance[0] = getDistance();

 //look right and get distance
  servoMotor.write(RIGHT_ANGLE);
  delay(300);
  distance[2] = getDistance();
  
  //look straight and get distance
  servoMotor.write(90);
  delay(300);
  distance[1] = getDistance();
  
  boolean obstacle = (distance[0] < TIGGER_DIST) or (distance[1] < TIGGER_DIST) or (distance[2] < TIGGER_DIST); // true iff any distance is less than minimum safe distance
  
  if (obstacle) {
    delay(90);
    moveBackward(400); //move back for 400 ms
    delay(750);
    
    //Look Left and get distance
    servoMotor.write(175);
    delay(300);
    int leftDistance = getDistance();
    
    if (leftDistance <= TIGGER_DIST){
      //Look Right
      servoMotor.write(5);
      delay(300);
      int rightDistance = getDistance();

      if (rightDistance <= TIGGER_DIST)moveBackward(400);
      
      turnRight(250);
      }
    else turnLeft(250);
  }
}//end loop


int getDistance(){
  /*Returns the distance from the sensor to the nearest object in cm
   * MAX DIST = 400 cm
   * MIN DIST = 2 cm
   */
  int dist = 0;
  dist = sonar.ping_cm();
  delay(275);
  if(dist == 0) return 20;
  return dist;
  }


//####################MOTOR MOVEMENT METHODS#######################
void setupMotorPins(){
  pinMode(INPUT1_PIN, OUTPUT);
  pinMode(INPUT2_PIN, OUTPUT);
  pinMode(INPUT3_PIN, OUTPUT);
  pinMode(INPUT4_PIN, OUTPUT);
  pinMode(ENABLE_A, OUTPUT);
  pinMode(ENABLE_B, OUTPUT);

  analogWrite(ENABLE_A,115); //for some reason,right motor is a bit weak
  analogWrite(ENABLE_B,100);
  
  }
void rightForward(){
  /*Moves the Right Motor Fowrward
   */
  digitalWrite(INPUT1_PIN, HIGH);
  digitalWrite(INPUT2_PIN, LOW);
  }  
void leftForward(){
   /*Moves the Left Motor Fowrward
   */
  digitalWrite(INPUT3_PIN, HIGH);
  digitalWrite(INPUT4_PIN, LOW); 
  }  
void rightBackward(){
   /*Moves the Right Motor Backward
   */
  digitalWrite(INPUT1_PIN, LOW);
  digitalWrite(INPUT2_PIN, HIGH);
  }  
void leftBackward(){
   /*Moves the Left Motor Backward
   */
  digitalWrite(INPUT3_PIN, LOW);
  digitalWrite(INPUT4_PIN, HIGH);
  }
void stopMovement(){
  digitalWrite(INPUT1_PIN, LOW);
  digitalWrite(INPUT2_PIN, LOW);
  digitalWrite(INPUT3_PIN, LOW);
  digitalWrite(INPUT4_PIN, LOW);  
  }
  
void moveForward(int moveTime){
  /*Moves forward
   * @args moveTime: move for moveTime miliseconds
   * @args moveTime: stop for stopTime miliseconds
   */
  leftForward();
  rightForward();
  delay(moveTime);
  stopMovement();
  }
void moveBackward(int moveTime){
    /*Moves backward
   * @args moveTime: move for moveTime miliseconds
   * @args moveTime: stop for stopTime miliseconds
   */
  leftBackward();
  rightBackward();
  delay(moveTime);
  stopMovement();
  }
void turnRight(int moveTime){
    /*Turn Right
   * @args moveTime: move for moveTime miliseconds
   * @args moveTime: stop for stopTime miliseconds
   */
  rightBackward();
  leftForward();
  delay(moveTime);
  stopMovement();
  }
void turnLeft(int moveTime){
    /*Turn Left
   * @args moveTime: move for moveTime miliseconds
   * @args moveTime: stop for stopTime miliseconds
   */
  leftBackward();
  rightForward();
  delay(moveTime);
  stopMovement();
  }
  
