#include <PID_v1.h>
#include <Servo.h>

int claw_position;
Servo Claw_Servo;

double wrist_setpoint, wrist_input, wrist_output;
PID Wrist_PID(&wrist_input, &wrist_output, &wrist_setpoint,.26, .888, .0002, REVERSE);
Servo Wrist_Servo;
int wrist_pot = A0;

double elbow_setpoint, elbow_input, elbow_output;
PID Elbow_PID(&elbow_input, &elbow_output, &elbow_setpoint,.26, .888, .0002, REVERSE);
Servo Elbow_Servo;
int elbow_pot = A1;

double shoulder_setpoint, shoulder_input, shoulder_output;
PID Shoulder_PID(&shoulder_input, &shoulder_output, &shoulder_setpoint,.26, 0.888, .0002, REVERSE);
Servo Shoulder_Servo;
int shoulder_pot = A2;

//double base_setpoint, base_input, base_output;
//PID Base_PID(&base_input, &base_output, &base_setpoint,.26, 0.888, 0.0002, REVERSE);
//Servo Base_Servo;
//int base_pot = A3;


void setup() {
 Serial.begin(9600);
 
 Claw_Servo.attach(12);
 claw_position = 130;
 
 Wrist_Servo.attach(3);
 Wrist_PID.SetOutputLimits(0, 176);
 pinMode(wrist_pot, INPUT);
 Wrist_PID.SetMode(AUTOMATIC);
 wrist_setpoint = 600;
 
 Elbow_Servo.attach(5);
 Elbow_PID.SetOutputLimits(0, 180);
 pinMode(elbow_pot, INPUT);
 Elbow_PID.SetMode(AUTOMATIC);
 elbow_setpoint=700;
 
 Shoulder_Servo.attach(6);
 Shoulder_PID.SetOutputLimits(20,160);
 pinMode(shoulder_pot, INPUT);
 Shoulder_PID.SetMode(AUTOMATIC);
 shoulder_setpoint = 730;
 
 //Base_Servo.attach(11);
 //Base_PID.SetOutputLimits(0,180);
 //pinMode(base_pot, INPUT);
 //Base_PID.SetMode(AUTOMATIC);
 //base_setpoint = 700;
 
}

void loop()
{
  
  if (Serial.available()){
    wrist_setpoint=map(Serial.parseInt(), 0, 180, 365, 1109);
    elbow_setpoint=map(Serial.parseInt(), 0 , 180, 283, 977);
    shoulder_setpoint=map(Serial.parseInt(), 0, 180, 1092, 353);
    //base_setpoint=map(Serial.parseInt(), 0, 180, 209, 930);
    claw_position=Serial.parseInt();
    Serial.println("Setpoints changed!");
  }

  Claw_Servo.write(claw_position);
  
  wrist_input = analogRead(wrist_pot);
  Wrist_PID.Compute();
  Wrist_Servo.write(wrist_output);
  //Serial.print(wrist_input); Serial.print("\t\t\t"); Serial.println(wrist_output);
  
  elbow_input = analogRead(elbow_pot);
  Elbow_PID.Compute();
  Elbow_Servo.write(elbow_output);
  //Serial.print(elbow_input); Serial.print("\t\t\t"); Serial.println(elbow_output);
  
  shoulder_input = analogRead(shoulder_pot);
  Shoulder_PID.Compute();
  Shoulder_Servo.write(shoulder_output);
  Serial.print(shoulder_input); Serial.print("\t\t\t"); Serial.println(shoulder_output);
  
  //base_input = analogRead(base_pot);
  //Base_PID.Compute();
  //Base_Servo.write(base_output);
  //Serial.print(base_input); Serial.print("\t\t\t"); Serial.println(base_output);
  
  
}
