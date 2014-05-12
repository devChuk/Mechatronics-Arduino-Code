#include <PID_v1.h>

#include <Servo.h>
#include <SPI.h>
#include <WiFly.h>

//Servos
Servo Shoulder_Motor;
Servo Elbow_Motor;
Servo Wrist_Motor;
Servo Gripper_Motor;

//PID Controllers
double shldrSetpoint, shldrInput, shoulder_output;
double elbowSetpoint, elbowInput, elbow_output;
double wristSetpoint, wristInput, wrist_output;
PID Shoulder_PID(&shldrInput, &shoulder_output, &shldrSetpoint,.01, 0.2, 0, REVERSE);
PID Elbow_PID(&elbowInput, &elbow_output, &elbowSetpoint,.02, 0.5, 0, REVERSE);
PID Wrist_PID(&wristInput, &wrist_output, &wristSetpoint,.02, 0.3, 0, REVERSE);

int shldrAngle;
int elbowAngle;
int wristAngle;
int time;
//Pins
int wristPot = A5;
int elbowPot = A4;
int shldrPot = A3;

//incoming messages
#define bufferlength 17
char c;
char inputbuffer[bufferlength];
byte index = 0;
int value = 1500;

//constants
const float A = 5.75;  //shoulder to elbow
const float B = 7.375; //elbow to wrist
const float C = 3.375; //wrist to gripper
const float rtod = 57.295779;

//Arm Start Pos
int Base, Gripper = 1500;
float Shoulder, Elbow, Wrist = 1500;
float X = 7.5;
float Y = -2.5;
float WA = -90;


boolean error;

void setup()  
{
  Serial.begin(9600);
  Serial.println("Robot_Arm_WiFi");
  Shoulder_Motor.attach(3);
  Elbow_Motor.attach(4);
  Wrist_Motor.attach(5);
  Gripper_Motor.attach(6);
  /*Shoulder_Motor.writeMicroseconds(1500);
  Elbow_Motor.writeMicroseconds(1500);
  Wrist_Motor.writeMicroseconds(1500);
  Gripper_Motor.writeMicroseconds(1500);*/
  Wrist_PID.SetOutputLimits(0, 176);
  Wrist_PID.SetMode(AUTOMATIC);
  wristSetpoint = 600;
  Elbow_PID.SetOutputLimits(0, 180);
  Elbow_PID.SetMode(AUTOMATIC);
  elbowSetpoint=700;
  Shoulder_PID.SetOutputLimits(20,160);
  Shoulder_PID.SetMode(AUTOMATIC);
  shldrSetpoint = 730;
  SpiSerial.begin();
}

void loop() // run over and over
{
  checkForInput();
  updateSensors();
  Shoulder_PID.Compute();
  Shoulder_Motor.write(shoulder_output);
  Elbow_PID.Compute();
  Elbow_Motor.write(elbow_output);
  Wrist_PID.Compute();
  Wrist_Motor.write(wrist_output);
}

void checkForInput(){
  if(Serial.available()) { // Outgoing data
    do {
      while(Serial.available() == 0);
      c = Serial.read();
      if (c == '?'){
        Serial.print("X: ");
        Serial.print(X);
        Serial.print("\t\tY: ");
        Serial.print(Y);
        Serial.print("\tE: ");
        Serial.println((int)error);
        break;
      }
      inputbuffer[index] = c;
    }
    while (++index < bufferlength && c != '\n');
    inputbuffer[index] = 0;
    index = 0;
    HandleCMD(inputbuffer, index);
  }
  if(SpiSerial.available()) { // Outgoing data
    do {
      while(SpiSerial.available() == 0);
      c = SpiSerial.read();
      inputbuffer[index] = c;
    }
    while (++index < bufferlength && c != '\n');
    Serial.print(inputbuffer);
    inputbuffer[index] = 0;
    index = 0;
    HandleCMD(inputbuffer, index);
  }
}

void HandleCMD(char* input, int length) {
  float value = 0;
  value = atof(&input[1]);  // calculate number following command
  if(input[0] == 'B') {        //Determine which var to adjust
    //Base.writeMicroseconds((int) value);
  }
  else if(input[0] == 'S') {
    //shldrSetpoint = value;
    shldrSetpoint = map(value, 0, 180, 1092, 353);
  }
  else if(input[0] == 'E') {
    //elbowSetpoint = value;
    elbowSetpoint = map(value, 0, 180, 283, 977);
  }
  else if(input[0] == 'W') {
    //wristSetpoint = value;
    wristSetpoint = map(value, 0, 180, 365, 1109);
  }
  else if(input[0] == 'G') {
    Gripper = map(value, 0, 180, 600, 2400);
    Gripper_Motor.writeMicroseconds(Gripper);
  }
  else if(input[0] == 'X') {
    X=value;
    Arm(X, Y, WA);
  }
  else if(input[0] == 'Y') {
    Y=value;
    Arm(X, Y, WA);
  }
  else if(input[0] == 'A') {
    WA=value;
    Arm(X, Y, WA);
  }
}

int Arm(float x, float y, float wa) { //Here's all the Inverse Kinematics to control the arm
  if(y==0 || x==0) {
    error = 1;
    Serial.println("ERROR!");
    return 0;
  }
  x = x - C*cos(wa*(3.14/180));
  y = y - C*sin(wa*(3.14/180));
  float M = sqrt((y*y)+(x*x));
  if(M <= 0 || x <= 0 || y==0) {
    error = 1;
    Serial.println("ERROR!");
    return 0;
  }
  float A1 = atan(y/x);
  float A2 = acos((A*A-B*B+M*M)/((A*2)*M));
  //calculate shoulder angle
  Shoulder = A1 + A2;
  Shoulder = Shoulder * rtod;
  //calculate elbow angle
  Elbow = acos((A*A+B*B-M*M)/((A*2)*B));
  Elbow = Elbow * rtod;
  //calculate wrist angle
  //Wris = abs(wa - Elbow - Shoulder) - 90;
  Wrist = 90 - (Shoulder-(180-Elbow)) + wa;
  if((int)Elbow <= 0 || (int)Shoulder <= 0 || (int)Wrist <= 0) {
    error = 1;
    Serial.println("ERROR!");
    return 0;
  }
  shldrSetpoint = map(Shoulder, 0, 180, 1092, 353);
  elbowSetpoint = map(Elbow, 0, 180, 283, 977);
  wristSetpoint = map(Wrist, 0, 180, 1109, 365);
  error = 0;
  return 1;
}

void updateSensors(){
  shldrInput = analogRead(A3);
  elbowInput = analogRead(A4);
  wristInput = analogRead(A5);
  shldrAngle=map(analogRead(A3), 1092, 353, 0, 180);
  elbowAngle=map(analogRead(A4), 283, 977, 0, 180);
  wristAngle=map(analogRead(A5), 365, 1109, 0, 180);
  Serial.print("S: ");
  Serial.print(shldrAngle);
  Serial.print("\t\tE: ");
  Serial.print(elbowAngle);
  Serial.print("\t\tW: ");
  Serial.println(wristAngle);
/*  if (millis()- time > 10000){
    SpiSerial.print('S');
    SpiSerial.print(shldrAngle);
    SpiSerial.print('\n');
    SpiSerial.print('E');
    SpiSerial.print(elbowAngle);
    SpiSerial.print('\n');
    SpiSerial.print('W');
    SpiSerial.print(wristAngle);
    SpiSerial.print('\n');
    time = millis();
  }*/
}
