#include<math.h>
#include <Servo.h>
#include <NewPing.h>
#include <QTRSensors.h>

#define TRIGGER_PIN  12  
#define ECHO_PIN     11  
#define MAX_DISTANCE 200 
#define CLOSE_SERVO_POS 20
#define OPEN_SERVO_POS 80
#define OPEN_SERVO_POS_AT_THE_POLE 90
#define CATCHED_SERVO_POS 20
#define NUM_SENSORS   6     
#define TIMEOUT       2500  
#define EMITTER_PIN   10   
#define Kp 0.1 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 2 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 220 // max speed of the robot
#define leftMaxSpeed 220 // max speed of the robot
#define rightBaseSpeed 200 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 200  // this is the speed at which the motors should spin when the robot is perfectly on the line  
#define POLEBASE 4500

QTRSensorsRC qtrrc((unsigned char[]) {43,45,47,49,51,53},NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
Servo servoHand;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int lEncoderCount;
int rEncoderCount;
int lastError = 0;
unsigned int sensorValues[NUM_SENSORS];

void setup() {
Serial.begin(9600);
delay(100);
// set Motor PINS  
  for(int i=0;i<6;i++){
    pinMode(2+i,OUTPUT);
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
//BEEP
  pinMode(52,OUTPUT);
//Sensor Panel Calibration
 sensorCalibrate();
// LImits
  pinMode(31,INPUT);
  pinMode(33,INPUT);
  pinMode(35,INPUT);

}

void loop() {
 firstAttempt();
 secondAttempt();
 thirdAttempt();
 finalAttempt();
 while(1);
}

void finalAttempt(){

  //got to 15 step reverse
  doEncorderRev(15);
  //stop motor
  motorBrake();
  motorStop();
  delay(200);
 turn180(); 
 //goto cross (C) position
  while(1){
    int p=getPosition();
    if(p==2500){
     break; 
    }
    doPID(); 
  }
  // pass the Mid point
  beep(1);
  doEncorder(35); 
  //goto cross start Line position
  while(1){
    int p=getPosition();
    if(p==2500){
     break; 
    }
    doPID(); 
  }
  // goto in the cage
  doEncorder(30);
  // motor Stop 
  motorBrake();
  motorStop();
  delay(200);
  beep(2);
  // final attempt is done. came to the start position 
}

void thirdAttempt(){
   //got to 15 step reverse
  doEncorderRev(15);
  //stop motor
  motorBrake();
  motorStop();
  delay(200);
  // turn 180 digries right
  turn180();
  //goto cross (C) position
  while(1){
    int p=getPosition();
    if(p==2500){
     break; 
    }
    doPID(); 
  }
  // pass the Mid point
  doEncorder(23);
  // stop motor
  motorBrake();
  motorStop();
  delay(200);
  beep(1);
  // turn 45 left
  turn45Left();
  // stop motor
  motorBrake();
  motorStop();
  delay(200);
  // goto the pole
  while(1){
    double d=getSonar();
    if(d==13 || d==12 ||d==11)
      break;
    doPID();
  }
  //stop motor
  motorBrake();
  motorStop();
  delay(200);
  beep(1);
  // slow run to the pole
  doEncorder(8,120);
  // stop the motor
  motorBrake();
  motorStop();
  delay(200);
  // down the hand to the pole(2cm up)
  vMotorDownOnThePole();
  delay(100);
  // close the hand catch the ring
  closeHand();
  delay(100);
  // go up the hand
  vMotorUp();
  delay(100);
  beep(1);
  // reverse the sudda 15 stepts
  doEncorderRev(15);
  // stop the motor
  motorBrake();
  motorStop();
  delay(200);
  // turn 180 
  turn180();
  // move sudda till to reach the cross(C)
  while(1){
    int p=getPosition();
    if(p==2500){
     break; 
    }
    doPID(); 
  }
  // pass the cross
  doEncorder(23);
  // stop the motor
  motorBrake();
  motorStop();
  delay(200);
  // turn right
  turn45Right();
  // stop the motor
  motorBrake();
  motorStop();
  delay(200);
  // follow the Line till meet the Pole
  while(1){
    double d=getSonar();
    if(d==13 || d==12||d==11)
      break;
    doPID();
  }
  //stop motor
  motorBrake();
  motorStop();
  delay(200);
  // slow run to the Pole
  doEncorder(11,120);
  //stop the sudda
  motorBrake();
  motorStop();
  delay(200);
  //release the Ring
  openHand();
  beep(3);
  // third Attempt is done.
}

void secondAttempt(){
  //got to 15 step reverse
  doEncorderRev(15);
  //stop motor
  motorBrake();
  motorStop();
  delay(200);
  // turn 180 digries right
  turn180();
  //goto cross (C) position
  while(1){
    int p=getPosition();
    if(p==2500){
     break; 
    }
    doPID(); 
  }
  // pass the Mid point
  doEncorder(23);
  // stop motor
  motorBrake();
  motorStop();
  delay(200);
  beep(1);
  // turn 45 right
  turn45Right();
  // stop motor
  motorBrake();
  motorStop();
  delay(200);
  // goto the pole
  while(1){
    double d=getSonar();
    if(d==13 || d==12 ||d==11)
      break;
    doPID();
  }
  //stop motor
  motorBrake();
  motorStop();
  delay(200);
  beep(1);
  // slow run to the pole
  doEncorder(8,120);
  // stop the motor
  motorBrake();
  motorStop();
  delay(200);
  // down the hand to the pole(2cm up)
  vMotorDownOnThePole();
  delay(100);
  // close the hand catch the ring
  closeHand();
  delay(100);
  // go up the hand
  vMotorUp();
  delay(100);
  beep(1);
  // reverse the sudda 15 stepts
  doEncorderRev(15);
  // stop the motor
  motorBrake();
  motorStop();
  delay(200);
  // turn 180 
  turn180();
  // move sudda till to reach the cross(C)
  while(1){
    int p=getPosition();
    if(p==2500){
     break; 
    }
    doPID(); 
  }
  // pass the cross
  doEncorder(23);
  // stop the motor
  motorBrake();
  motorStop();
  delay(200);
  // turn left
  turn45Left();
  // stop the motor
  motorBrake();
  motorStop();
  delay(200);
  // follow the Line till meet the Pole
  while(1){
    double d=getSonar();
    if(d==13 || d==12||d==11)
      break;
    doPID();
  }
  //stop motor
  motorBrake();
  motorStop();
  delay(200);
  // slow run to the Pole
  doEncorder(11,120);
  //stop the sudda
  motorBrake();
  motorStop();
  delay(200);
  //release the Ring
  openHand();
  beep(3);
  // second Attempt is done.
}

void firstAttempt(){
 // down the hand 
 vMotorDown();
 delay(100);
 doEncorder(10);
  // do pid for find the ring
  while(1){
    double d=getSonar();
    if(d==13 || d==12 ||d==11)
      break;
    doPID();
  }
  //stop motor
  motorBrake();
  motorStop();
  delay(200);
  beep(1);
  // down the hand
  vMotorDown();
  beep(1);
  delay(10);
  //go Encorder 
  doEncorder(20);
  motorBrake();
  motorStop();
  delay(200);
  //close htch
  closeHand();
  beep(1);
  delay(10);
  // up the hand
  vMotorUp();
  delay(100);
  //jumpOut the cross
  doEncorder(20);
  // follow the Line till meet the Pole
  while(1){
    double d=getSonar();
    if(d==13 || d==12||d==11)
      break;
    doPID();
  }
  //stop motor
  motorBrake();
  motorStop();
  delay(200);
  // slow run to the Pole
  doEncorder(11,120);
  //stop the sudda
  motorBrake();
  motorStop();
  delay(200);
  //release the Ring
  openHand();
  beep(3);
  // first Attempt is done.
}

void doEncorder(int enc){
  rEncoderCount=0;
  lEncoderCount=0;
  while(1){
    if(lEncoderCount <= enc){
      lMotorForward(150);
    }else{
      lMotorForward(0);
    }
    
    if(rEncoderCount <= enc)
    {
      rMotorForward(150);  
    }else{
      rMotorForward(0);
    }
    
    if(rEncoderCount>enc && lEncoderCount>enc)
      break;
  }
  motorBrake();
  motorStop();
}

void doEncorder(int enc,int speed_){
  rEncoderCount=0;
  lEncoderCount=0;
  while(1){
    if(lEncoderCount <= enc){
      lMotorForward(speed_);
    }else{
      lMotorForward(0);
    }
    
    if(rEncoderCount <= enc)
    {
      rMotorForward(speed_);  
    }else{
      rMotorForward(0);
    }
    
    if(rEncoderCount>enc && lEncoderCount>enc)
      break;
  }
  motorBrake();
  motorStop();
}

void doEncorderRev(int enc){
  rEncoderCount=0;
  lEncoderCount=0;
  while(1){
    if(lEncoderCount <= enc){
      lMotorReverse(150);
    }else{
      lMotorReverse(0);
    }
    
    if(rEncoderCount <= enc)
    {
      rMotorReverse(150);  
    }else{
      rMotorReverse(0);
    }
    
    if(rEncoderCount>enc && lEncoderCount>enc)
      break;
  }
  motorBrake();
  motorStop();
}

void doPID(){
  int position = getPosition(); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - 2500;

  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed - motorSpeed;
  int leftMotorSpeed = leftBaseSpeed + motorSpeed;
  
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  motorForward(leftMotorSpeed,rightMotorSpeed);
}

int vMotorIsUp(){
  int y1=digitalRead(31);
  return y1;  
}

int vMotorIsDown(){
  int y1=digitalRead(33);
  return y1;  
}

int vMotorIsDownOnThePole(){
  int y1=digitalRead(35);
  return y1;  
}

void vMotorDown(){
  digitalWrite(24,HIGH);
  digitalWrite(22,LOW);
  while(vMotorIsDown()==1);
  motorVStop();  
}

void vMotorDownOnThePole(){
  digitalWrite(24,HIGH);
  digitalWrite(22,LOW);
  while(vMotorIsDownOnThePole()==1);
  motorVStop();  
}

void vMotorDown(int time_){
  digitalWrite(24,HIGH);
  digitalWrite(22,LOW);
  delay(time_);
  motorVStop();  
}

void vMotorUp(){
  digitalWrite(22,HIGH);
  digitalWrite(24,LOW);
  while(vMotorIsUp()==1);
  motorVStop();  
}

void vMotorUp(int time_){
  digitalWrite(22,HIGH);
  digitalWrite(24,LOW);
  delay(time_);
  motorVStop();  
}

int getPosition(){
  unsigned int position = qtrrc.readLine(sensorValues);
  return position;  
}

void sensorCalibrate(){
  delay(100);
  beep(1);
  for (int i = 0; i < 200; i++)  
  {
    qtrrc.calibrate();      
  }
  beep(2);   
  delay(1000);
}

//BEEEP
void beep(int i){
  for(int j=0;j<i;j++){
    digitalWrite(52,HIGH);
  delay(100);
  digitalWrite(52,LOW);
  delay(100);   
  }
}

//get the sonar value
double getSonar(){
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  return uS / US_ROUNDTRIP_CM;
}

// ENCORDER
void initEncoders(){
  
  attachInterrupt(4, lEncoderIncrement, CHANGE);
  attachInterrupt(5, rEncoderIncrement, CHANGE);
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
// release the ring
void openHand(){
  for(int i=CATCHED_SERVO_POS;i<=OPEN_SERVO_POS;i++){
    servoHand.write(i);
    delay(5);
  }
}
//catch the ring
void closeHand(){
  for(int i=OPEN_SERVO_POS;i>=CATCHED_SERVO_POS;i--){
    servoHand.write(i);
    delay(5);
  }
}
// put the hnd on to the pole base
void openHandAtThePole(){
    for(int i=OPEN_SERVO_POS;i>=OPEN_SERVO_POS_AT_THE_POLE;i--){
      servoHand.write(i);
      delay(5);
    }
}

//******************** motor Movement************************

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

void lMotorReverse(int pwm){
  digitalWrite(3,HIGH);
  digitalWrite(4,LOW);
  analogWrite(2, pwm);
}

void rMotorReverse(int pwm){
  digitalWrite(6,HIGH);
  digitalWrite(5,LOW);
  analogWrite(7, pwm);
}

void motorReverse(int pwml,int pwmr){
  rMotorReverse(pwmr);
  lMotorReverse(pwml);  
}

void motorStop(){
  
  digitalWrite(4,LOW);
  digitalWrite(3,LOW);
  analogWrite(2, 0);
  digitalWrite(6,LOW);
  digitalWrite(5,LOW);
  analogWrite(7, 0);
  delay(30);
}

void motorBrake(){

  digitalWrite(4,HIGH);
  digitalWrite(3,HIGH);
  analogWrite(2, 255);
  digitalWrite(6,HIGH);
  digitalWrite(5,HIGH);
  analogWrite(7, 255);
  delay(30);
}
// *******************turnings**************************
void turn180(){
    rEncoderCount=0;
    while(rEncoderCount<96){
      lMotorReverse(200);
      rMotorForward(200);
   }
    motorBrake();
    motorStop();
    delay(100);
}

void turn180f(){
    rEncoderCount=0;
    while(rEncoderCount<116){
      lMotorReverse(200);
      rMotorForward(200);
   }
    motorBrake();
    motorStop();
    delay(100);
}

void turn45Right(){
    rEncoderCount=0;
    while(rEncoderCount<21){
      rMotorReverse(200);
      lMotorForward(200);
   }
    motorBrake();
    motorStop();
    delay(100);
}

void turn45Left(){
    rEncoderCount=0;
    while(rEncoderCount<19){
      lMotorReverse(200);
      rMotorForward(200);
   }
    motorBrake();
    motorStop();
    delay(100);
}

