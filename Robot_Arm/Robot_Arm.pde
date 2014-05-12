#include <Servo.h>
#include <WiFly.h>

#define bufferlength 15
char c;
char inputbuffer[bufferlength];
byte index = 0;

//uncomment for digital servos in the Shoulder and Elbow
//that use a range of 900ms to 2100ms
//#define DIGITAL_RANGE

const float A = 5.75;  //shoulder to elbow
const float B = 7.375; //elbow to wrist
const float C = 3.375; //wrist to gripper

//Arm Servo pins
#define Base_pin 2
#define Shoulder_pin 3
#define Elbow_pin 4
#define Wrist_pin 5
#define Gripper_pin 6

//Radians to Degrees constant
const float rtod = 57.295779;

//Servo Objects
Servo Elb;
Servo Shldr;
Servo Wrist;
Servo Base;
Servo Gripper;

//Arm Start Pos
float X = 7.5;
float Y = -2.5;
int Z = 40;
int G = 90;
float WA = -90;

float Shoulder;
float Elbow;
float Wris;

//Arm temp pos
float tmpS;
float tmpE;
float tmpW;

float incS;
float incE;
float incW;

boolean error;

void setup() {
  Serial.begin(9600);
  SpiSerial.begin();
  Base.attach(Base_pin);
  Shldr.attach(Shoulder_pin);
  Elb.attach(Elbow_pin);
  Wrist.attach(Wrist_pin);
  Gripper.attach(Gripper_pin);
  Arm(X, Y, Z, G, WA);
  tmpS = Shoulder;
  tmpE = Elbow;
  tmpW = Wris;
}

void loop() {
  //checkforinput();
  checkForWiFlyInput();
  Arm(X, Y, Z, G, WA);
  if (!error){
    SpiSerial.print("R0#");
    SpiSerial.print(byte(10));
    //Sweep();
    sendValues();
  } else {
    SpiSerial.print("R1#");
    SpiSerial.print(byte(10));
  }
}

void checkforinput() {
  if(Serial.available()) { // Outgoing data
    do {
      while(Serial.available() == 0);
      c = Serial.read();
      inputbuffer[index] = c;
    } 
    while (++index < bufferlength && c != '#');
    inputbuffer[index] = 0;
    index = 0;
    HandleCMD(inputbuffer, index);
  }
}

void checkForWiFlyInput() {
  if(SpiSerial.available()) { // Outgoing data
    do {
      while(SpiSerial.available() == 0);
      c = SpiSerial.read();
      /*if (c == '*') {
        OpenorClos();
        break;
      }*/
      inputbuffer[index] = c;
    } 
    while (++index < bufferlength && c != '#');
    inputbuffer[index] = 0;
    index = 0;
    Serial.println(inputbuffer);
    HandleCMD(inputbuffer, index);
  }
}

void HandleCMD(char* input, int length) {
  float value = 0;
  value = atof(&input[1]);  // calculate number following command
  if(input[0] == 'X') {        //Determine which var to adjust
    X=value;
    //Arm(X, Y, Z, G, WA);
  }
  else if(input[0] == 'Y') {
    Y=value;
    //Arm(X, Y, Z, G, WA);
  }
  else if(input[0] == 'W') {
    WA=value;
    //Arm(X, Y, Z, G, WA);
  }
  else if(input[0] == 'G') {
    Gripper.write(value);
  }
  
}

int Arm(float x, float y, float z, int g, float wa) { //Here's all the Inverse Kinematics to control the arm
  if(y==0 || x==0) {
    error = 1;
    return 0;
  }
  x = x - C*cos(wa*(3.14/180));
  y = y - C*sin(wa*(3.14/180));
  float M = sqrt((y*y)+(x*x));
  if(M <= 0 || x <= 0 || y==0) {
    error = 1;
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
  Wris = 90 - (Shoulder-(180-Elbow)) + wa;
  if((int)Elbow <= 0 || (int)Shoulder <= 0 || (int)Wris <= 0) {
    error = 1;
    return 0;
  }
  Serial.print("   S: ");
  Serial.print(Shoulder);
  Serial.print("\t\t   E: ");
  Serial.print(Elbow);
  Serial.print("\t\t   W: ");
  Serial.println(Wris);
  //increments();
  #ifdef DIGITAL_RANGE
  Shldr.writeMicroseconds(map(Shoulder, 0, 180, 900, 2100));
  Elb.writeMicroseconds(map(180 - Elbow, 0, 180, 900, 2100));
  #else
  Shldr.write(Shoulder);
  Elb.write(180 - Elbow);
  #endif
  Wrist.write(Wris);
  Base.write(Z);
  error = 0;
  return 1;
}

void increments() {
  float dShoulder = abs(Shoulder - tmpS);
  float dElbow = abs(Elbow - tmpE);
  float dWris = abs(Wris - tmpW);
  Serial.print("disS: ");
  Serial.print(dShoulder);
  Serial.print("\t\tdisE: ");
  Serial.print(dElbow);
  Serial.print("\t\tdisW: ");
  Serial.println(dWris);
  float maxdisp = max(dShoulder, max(dElbow, dWris));
  if (maxdisp == dShoulder) {
    incS = 5;
    incE = 5*(dElbow/dShoulder);
    incW = 5*(dWris/dShoulder);
  }
  else if (maxdisp == dElbow) {
    incS = 5*(dShoulder/dElbow);
    incE = 5;
    incW = 5*(dWris/dElbow);
  } else if (maxdisp == dWris) {
    incS = 5*(dShoulder/dWris);
    incE = 5*(dElbow/dWris);
    incW = 5;
  }
  Serial.print("incS: ");
  Serial.print(incS);
  Serial.print("\t\tincE: ");
  Serial.print(incE);
  Serial.print("\t\tincW: ");
  Serial.println(incW);
}

void Sweep() {
  Base.write(Z);
  if (Shoulder - tmpS > incS) {
    tmpS += incS;
  }
  else if (Shoulder - tmpS < -incS) {
    tmpS -= incS;
  }
  else {
    tmpS = Shoulder;
  }
  if (Elbow - tmpE > incE) {
    tmpE += incE;
  }
  else if (Elbow - tmpE < -incE) {
    tmpE -= incE;
  }
  else {
    tmpE = Elbow; 
  }
  if (Wris - tmpW > incW) {
    tmpW += incW;
  }
  else if (Wris - tmpW < -incW) {
    tmpW -= incW;
  }
  else {
    tmpW = Wris;
  }
#ifdef DIGITAL_RANGE
  Shldr.writeMicroseconds(map(tmpS, 0, 180, 900, 2100));
  Elb.writeMicroseconds(map(180 - tmpE, 0, 180, 900, 2100));
#else
  Shldr.write(tmpS);
  Elb.write(180 - tmpE);
#endif
  Wrist.write(tmpW);
}

void sendValues(){
  SpiSerial.print("S");
  SpiSerial.print(Shoulder);
  SpiSerial.print("#");
  SpiSerial.print(byte(10));
  SpiSerial.print("E");
  SpiSerial.print(Elbow);
  SpiSerial.print("#");
  SpiSerial.print(byte(10));
  SpiSerial.print("W");
  SpiSerial.print(Wris);
  SpiSerial.print("#");
  SpiSerial.print(byte(10));
}
