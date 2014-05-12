#include <Servo.h>

#define bufferlength 15
char c;
char inputbuffer[bufferlength];
byte index = 0;

//uncomment for digital servos in the Shoulder and Elbow
//that use a range of 900ms to 2100ms
//#define DIGITAL_RANGE

const float A = 5.75;
const float B = 7.375;

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

//Arm Current Pos
float X = 4;
float Y = 4;
int Z = 90;
int G = 90;
float WA = 0;

//Arm temp pos
float tmpx = 4;
float tmpy = 4;
int tmpz = 90;
int tmpg = 90;
float tmpwa = 0;

void setup() {
  Serial.begin(9600);
  Base.attach(Base_pin);
  Shldr.attach(Shoulder_pin);
  Elb.attach(Elbow_pin);
  Wrist.attach(Wrist_pin);
  Gripper.attach(Gripper_pin);
}

void loop() {
  checkforinput();
  Arm(tmpx, tmpy, tmpz, tmpg, tmpwa);
  Serial.println(Base.read());
  Serial.println(Shldr.read());
  Serial.println(Elb.read());
  Serial.println(Wrist.read());
  Serial.println(Gripper.read());
  delay(1500);
}

void checkforinput(){
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

void HandleCMD(char* input, int length) {
  float value = 0;
  value = atof(&input[1]);  // calculate number following command
  if(input[0] == 'X') {        //Determine which var to adjust
    tmpx=value;
  }
  else if(input[0] == 'Y') {
    tmpy=value;
  }
}

int Arm(float x, float y, float z, int g, float wa) { //Here's all the Inverse Kinematics to control the arm
  float M = sqrt((y*y)+(x*x));
  if(M <= 0 || x <= 0) {
    return 1;
  }
  float A1 = atan(y/x);
  float A2 = acos((A*A-B*B+M*M)/((A*2)*M));
  //calculate shoulder angle
  float Shoulder = A1 + A2;
  Shoulder = Shoulder * rtod;
  //calculate elbow angle
  float Elbow = acos((A*A+B*B-M*M)/((A*2)*B));
  Elbow = Elbow * rtod;
  
  if((int)Elbow <= 0 || (int)Shoulder <= 0) {
    return 1;
  }
  //calculate wrist angle
  float Wris = abs(wa - Elbow - Shoulder) - 90;
  //write to motors
  Base.write(z);
#ifdef DIGITAL_RANGE
  Shldr.writeMicroseconds(map(Shoulder, 0, 180, 900, 2100));
  Elb.writeMicroseconds(map(180 - Elbow, 0, 180, 900, 2100));
#else
  Shldr.write(Shoulder);
  Elb.write(180 - Elbow);
#endif
  Wrist.write(180 - Wris);
  Gripper.write(g);
  return 0;
}
