#include<math.h>
#include <Servo.h>
#include <NewPing.h>
#include <QTRSensors.h>

#define TRIGGER_PIN  12  
#define ECHO_PIN     11  
#define MAX_DISTANCE 200 
#define CLOSE_SERVO_POS 65
#define OPEN_SERVO_POS 140
#define CATCHED_SERVO_POS 65
#define NUM_SENSORS   6     
#define TIMEOUT       2500  
#define EMITTER_PIN   2     

QTRSensorsRC qtrrc((unsigned char[]) {43,45,47,49,51,53},NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];


Servo servoHand;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int lEncoderCount;
int rEncoderCount;


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
}

void loop() {
unsigned int position = qtrrc.readLine(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(position); // comment this line out if you are using raw values
  
  delay(250);
}

int vMotorIsUp(){
  int y1=digitalRead(31);
  return y1;  
}

int vMotorIsDown(){
  int y1=digitalRead(33);
  return y1;  
}

void vMotorDown(){
  digitalWrite(24,HIGH);
  digitalWrite(22,LOW);
  while(vMotorIsDown()==1);
  motorVStop();  
}

void vMotorUp(){
  digitalWrite(22,HIGH);
  digitalWrite(24,LOW);
  while(vMotorIsUp()==1);
  motorVStop();  
}

int getPosition(){
  unsigned int position = qtrrc.readLine(sensorValues);
  return position;  
}

void sensorCalibrate(){
  delay(100);
  beep(1);
  for (int i = 0; i < 400; i++)  
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

//
double sonarDistance(){
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

