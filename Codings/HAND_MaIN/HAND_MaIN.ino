#include<math.h>
#include <Servo.h>

#define CLOSE_SERVO_POS 65
#define OPEN_SERVO_POS 140
#define CATCHED_SERVO_POS 75

Servo servoHand;

int lEncoderCount;
int rEncoderCount;


void setup() {
Serial.begin(9600);
delay(100);
// set Motor PINS  
  for(int i=0;i<6;i++){
//    pinMode(2+i,OUTPUT);
  }

//set SensorPanel Pins
  for(int i=0;i<12;i+=2){
    pinMode(30+i,INPUT);
  }

  servoHand.attach(8);
// main v motor  
  pinMode(22,OUTPUT);
  pinMode(24,OUTPUT);
  motorVStop();

//ENCORDER init
  initEncoders();
}

void loop() {
  //motorForward(150,150);
  Serial.print(getLEncoderCount());
  Serial.print("  ");
  Serial.println(getREncoderCount());
}

// ENCORDER
void initEncoders(){
  
  attachInterrupt(4, lEncoderIncrement, CHANGE);
  //attachInterrupt(4, rEncoderIncrement, CHANGE);
}

void lEncoderIncrement(){
  lEncoderCount++;
}

void rEncoderIncrement(){
  rEncoderCount++;
}

int getLEncoderCount(){
  return lEncoderCount;
}

int getREncoderCount(){
  return  rEncoderCount;
}
//motor V controls
void motorVStop(){
  digitalWrite(22,HIGH);
  digitalWrite(24,HIGH);
  delay(100);
  digitalWrite(22,LOW);
  digitalWrite(24,LOW);
}

void openHand(){
  for(int i=CATCHED_SERVO_POS;i<=OPEN_SERVO_POS;i++){
    servoHand.write(i);
    delay(5);
  }
}

void closeHand(){
  for(int i=OPEN_SERVO_POS;i>=CATCHED_SERVO_POS;i--){
    servoHand.write(i);
    delay(5);
  }
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

