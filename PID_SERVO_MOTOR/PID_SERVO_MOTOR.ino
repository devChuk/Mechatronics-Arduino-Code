/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/


//FIND THE MAP NUMBERS.
//FINISH WIRING, MAKE SURE THERE IS NO SHORTING.


#include <PID_v1.h>
#include <Servo.h> 

//Set variables.
double wrist_setpoint, elbow_setpoint, shoulder_setpoint, base_setpoint;

double wrist_input, elbow_input, shoulder_input, base_input;

double wrist_output, elbow_output, shoulder_output, base_output;

//Specify the links and initial tuning parameters
PID Wrist_PID(&wrist_input, &wrist_output, &wrist_setpoint,.02, 0.4, 0, REVERSE);
PID Elbow_PID(&elbow_input, &elbow_output, &elbow_setpoint,.015, 0.1, 0, REVERSE);
PID Shoulder_PID(&shoulder_input, &shoulder_output, &shoulder_setpoint,.01, 0.01, 0, REVERSE);
PID Base_PID(&base_input, &base_output, &base_setpoint,.1, 0.7, 0, REVERSE);

//wrist: .02,.7,0,REVERSE
//Elbow: 0.05, .42, 0, REVERSE
//shoulder: .05 .42 .02 REVERSE
//base: .1, 0.7, 0 REVERSE
Servo Wrist_Servo, Elbow_Servo, Shoulder_Servo, Base_Servo, Claw_Servo;

//Set the potentiometer pins
int wrist_pot = A0;
int elbow_pot = A1;
int shoulder_pot = A2;
int base_pot = A3;

int claw_position;

void setup()
{
  //initialize the variables we're linked to
  Serial.begin(9600);
  Serial.print("PID");
  Serial.setTimeout(100);  //Sets the amount of time Serial.Timeout() waits until it cuts off the integer.
  Wrist_Servo.attach(3);   //Attach the servos.
  Elbow_Servo.attach(5);
  Shoulder_Servo.attach(6);
  Base_Servo.attach(11);
  Claw_Servo.attach(12);
  Wrist_PID.SetOutputLimits(0, 176);
  Elbow_PID.SetOutputLimits(0, 180);
  Shoulder_PID.SetOutputLimits(0, 180);
  Base_PID.SetOutputLimits(0, 180);
  pinMode(wrist_pot, INPUT);
  pinMode(elbow_pot, INPUT);
  pinMode(shoulder_pot, INPUT);
  pinMode(base_pot, INPUT);
  shoulder_setpoint=600;
  //wrist:0,176 (may be the same for others)
  //Elbow:
  //shoulder:
  //base:
  Wrist_PID.SetMode(AUTOMATIC);  //Initialize PID Loops.
  Elbow_PID.SetMode(AUTOMATIC);
  Shoulder_PID.SetMode(AUTOMATIC);
  Base_PID.SetMode(AUTOMATIC);
}

void loop()
{
  if (Serial.available()){
    wrist_setpoint=map(Serial.parseInt(), 0, 180, 365, 1109);
    elbow_setpoint=map(Serial.parseInt(), 0 , 180, 9, 753);
    shoulder_setpoint=map(Serial.parseInt(), 0, 180, 353, 1092);
    base_setpoint=map(Serial.parseInt(), 0, 180, 296, 970);
    claw_position=Serial.parseInt();
    Serial.println("Setpoints changed!");
  }
  wrist_input = analogRead(wrist_pot);
  elbow_input = analogRead(elbow_pot);
  shoulder_input = analogRead(shoulder_pot);
  base_input = analogRead(base_pot);
  Wrist_PID.Compute();
  Elbow_PID.Compute();
  Shoulder_PID.Compute();
  Base_PID.Compute();
  Serial.print(shoulder_input);
  Serial.print("\t\t\t");
  Serial.println(shoulder_output);
  //Wrist_Servo.write(wrist_output);
  //Elbow_Servo.write(elbow_output);
  Shoulder_Servo.write(shoulder_output);
  //Base_Servo.write(base_output);
  //Claw_Servo.write(claw_position);
}
