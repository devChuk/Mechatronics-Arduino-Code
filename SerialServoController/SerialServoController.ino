#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3);

#define bufferlength 17
char c;
char inputbuffer[bufferlength];
int value = 0;
byte index = 0;
String msg;
String pos;

void setup()  
{
  Serial.begin(9600);
  Serial.println("SerialServoController Example");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(115200);
  //mySerial.println("Hello, world?");
}

void loop() // run over and over
{
  value = checkForInput();
  mySerial.print("#3P");
  mySerial.print(value);
  mySerial.print("T1000\r");
}

int checkForInput(){
  if(Serial.available()) { // Outgoing data
    do {
      while(Serial.available() == 0);
      c = Serial.read();
      inputbuffer[index] = c;
    } 
    while (++index < bufferlength && c != '\n');
    inputbuffer[index] = 0;
    value = atoi(&inputbuffer[0]);
    Serial.println(value);
    index = 0;
  }
  return value;
}
