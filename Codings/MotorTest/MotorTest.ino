#include<math.h>
#include <Servo.h>

#define CLOSE_SERVO_POS 65
#define OPEN_SERVO_POS 140
#define CATCHED_SERVO_POS 80

Servo servoHand;

void setup() {

// set Motor PINS  
  for(int i=0;i<6;i++){
    pinMode(2+i,OUTPUT);
  }

//set SensorPanel Pins
  for(int i=0;i<12;i+=2){
    pinMode(30+i,INPUT);
  }

  servoHand.attach(8);
  
}

void loop() {
 servoHand.write(OPEN_SERVO_POS);
 delay(4000);
  for(int i=OPEN_SERVO_POS;i>=CATCHED_SERVO_POS;i--){
    servoHand.write(i);
    delay(15);
  }
  
 delay(4000);

}

void lMotorForward(int pwm){
  digitalWrite(4,HIGH);
  digitalWrite(3,LOW);
  analogWrite(2, pwm);
}

void rMotorForward(int pwm){
  digitalWrite(5,HIGH);
  digitalWrite(6,LOW);
  analogWrite(7, pwm);
}

void motorForward(int pwml,int pwmr){
  rMotorForward(pwmr);
  lMotorForward(pwml);  
}

void motorStop(){
  
  digitalWrite(4,LOW);
  digitalWrite(3,LOW);
  analogWrite(2, 0);
  digitalWrite(6,LOW);
  digitalWrite(5,LOW);
  analogWrite(7, 0);
  
}

void motorBrake(){

  digitalWrite(4,HIGH);
  digitalWrite(3,HIGH);
  analogWrite(2, 255);
  digitalWrite(6,HIGH);
  digitalWrite(5,HIGH);
  analogWrite(7, 255);
  
}

