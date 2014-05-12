#include <SPI.h>
#include <WiFly.h>
#include <SoftwareSerial.h>

//incoming messages
#define bufferlength 17
char c;
char inputbuffer[bufferlength];
byte index = 0;
int value = 1500;

//servo controller
SoftwareSerial mySerial(2, 3);
String H = "#",P = "P", S = "S",  T = "T";
String msg;
int time = 500;
int rate = 500;

//Servo Pins
#define Base_chan 0
#define Shoulder_chan 1
#define Elbow_chan 2
#define Wrist_chan 3
#define Gripper_chan 4

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
  Serial.println("Robot_Arm_Servo_Cont");
  // set the data rate for the SoftwareSerial port
  mySerial.begin(115200);
  mySerial.print("#0P1500#1P1500#2P1550#3P1400#4P1500T1000\r");
  SpiSerial.begin();
}

void loop() // run over and over
{
  checkForInput();
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
    msg = H + Base_chan + P + (int) value + T + time + "\r";
    mySerial.print(msg);
  }
  else if(input[0] == 'S') {
    Shoulder = value;
    msg = H + Shoulder_chan + P + (int) value + T + time + "\r";
    mySerial.print(msg);
  }
  else if(input[0] == 'E') {
    Elbow = value;
    msg = H + Elbow_chan + P + (int) value + T + time + "\r";
    mySerial.print(msg);
  }
  else if(input[0] == 'W') {
    Wrist = value;
    msg = H + Wrist_chan + P + (int) value + T + time + "\r";
    mySerial.print(msg);
  }
  else if(input[0] == 'G') {
    Gripper = map(value, 0, 180, 600, 2400);
    msg = H + Gripper_chan + P + Gripper + T + 10 + "\r";
    mySerial.print(msg);
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
  Serial.print("S: ");
  Serial.print(Shoulder);
  Serial.print("\t\tE: ");
  Serial.print(Elbow);
  Serial.print("\t\tW: ");
  Serial.println(Wrist);
  Shoulder = map(Shoulder, 0, 180, 600, 2400);
  Elbow = map(Elbow, 0, 180, 2400, 600);
  Wrist = map(Wrist, 0, 180, 600, 2400);
  msg = H + Shoulder_chan + P + (int) Shoulder + S + rate + H + Elbow_chan + P + (int) Elbow + S + rate + H + Wrist_chan + P + (int) Wrist + S + rate + T + 10 + "\r";
  mySerial.print(msg);
  error = 0;
  return 1;
}
