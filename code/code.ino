//This code is based on starter code provided by Elegoo
//www.elegoo.com

#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo

//Define all of the pins used.  These are NOT up to us, but rather what Elegoo decided.  Don't change.
int Echo = A4;  
int Trig = A5; 

float distance = 200;
float minDistance = 0;
int servoAngle = 90;

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define LED 13
//Line follower pins
#define LR 10
#define LM 4
#define LL 2

/******************************Helper functions*********************************************/
//Begin helper functions.  You should CALL these functions, but do not change them.  You DO NOT need to worry about the details inside the functions.

//The functions below set the left and right motor speeds and directions.
//Speed is an integer from 0 - 255 that determines the motor speed.
//Direction is 1 for forward and 0 for backwards.
void leftMotor(int lspeed, bool ldirection){
  analogWrite(ENA, lspeed);
  if (ldirection)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}
void rightMotor(int rspeed, bool rdirection){
  analogWrite(ENB, rspeed);
  if (rdirection)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}
//  The function below stops.  This is the same as just setting motor speeds to zero - not really necessary
void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
} 

//Ultrasonic distance measurement helper function - returns a float with distance in cm
float Distance_test() {
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance / 58.0;       
  return Fdistance;
}  

/*************************Setup*************************************************/
//You shouldn't need to touch this - it is merely setting up pins and stopping the motors at the start
void setup() { 
  myservo.attach(3,700,2400);  // attach servo on pin 3 to servo object
  Serial.begin(9600);     //For debugging
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LED, OUTPUT);
  stop();
} 

//Drives the robot
void drive(int leftSpeed = 200, int duration = 200, int rightSpeed = 200, bool leftDir = 1, bool rightDir = 1, bool brake = true){
  leftMotor(leftSpeed, leftDir);
  rightMotor(rightSpeed, rightDir);
  delay(duration);
  if(brake){
    stop();
    delay(100);
  }
}

float servoMoveAndMeasureDist(int angle){
  myservo.write(angle);
  distance = Distance_test();
  delay(100);
  return(distance);
}

bool isTimerAndCond(bool conditional = true){
  if(millis() > 2000){
    if(conditional){
      return(true);
    }
    else{
      return(false);
    }
  }
  else{
    return(false);
  }
}
/********************************Loop - your's to edit!****************************************************************************/
//Below is some skeleton code that calls functions.  Your primary task is to edit this code to accomplish your goals.
void loop() {
    myservo.write(servoAngle);  
    delay(500); 
    while(isTimerAndCond(distance > 150)){
      drive(200, 100, 100);
      distance = Distance_test();  
      Serial.println(distance);
    }
    //Detected the can
    minDistance = distance + 50;
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);

    while(isTimerAndCond(distance > 10)){
      distance = Distance_test();
      if(distance > minDistance){
        while(isTimerAndCond(((distance > minDistance) or (servoAngle < 180)))){
          servoAngle = servoAngle + 10;
          distance = servoMoveAndMeasureDist(servoAngle); 
        }
        while(isTimerAndCond(((distance > minDistance) or (servoAngle > 0)))){
          servoAngle = servoAngle - 10;
          distance = servoMoveAndMeasureDist(servoAngle);
        }
        if(servoAngle > 90){
          while(distance > minDistance){
            drive(100, 200, 100);
            distance = Distance_test();
            delay(100);
          }
        }
        else{
          while(isTimerAndCond(distance < minDistance)){
            drive(200, 100, 100);
            distance = Distance_test();
            delay(100);
          }
        }
        }
      else{
        drive(200, 200, 500);
      } 
    }
    while(1 == 1){
      stop();
    }
}
