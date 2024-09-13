
// Include the AccelStepper Library
#include <Servo.h>

// Define pin connections //Arduino UNO // Ardunio NANO
const int azDirPin = 7;   // 7          // 7
const int azStepPin = 4;  // 4          // 8
const int elDirPin = 5;   // 5          // 3
const int elStepPin = 2;  // 2          // 4

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance per stepper
AccelStepper azStepper(motorInterfaceType, azStepPin, azDirPin);
AccelStepper elStepper(motorInterfaceType, elStepPin, elDirPin);

// Define stepper reduction
const int azStepperReduction = 100;
const int elStepperReduction = 100;

// Variable for defining reduction
int targetReduction = 0;

// Define steps for 360 loop
const int stepsPerLoop = 200;

// Define serial comm constants
const byte numChars = 32;
char receivedChars[numChars];
int receivedLength;

// Define variable for received commands
char recvCmd[numChars/2];
char recvTargetMotor[numChars/8];
int recvTargetMotorLength;
int recvCmdLength;
char recvCmdData[numChars/2];

int recvCmdDataLength;
// Varialbe for received data
boolean newData = false;

void setup() {
  Serial.begin(115200);
  pinMode(9, OUTPUT);    // sets the digital pin 9 as output
  digitalWrite(9, LOW);
  
}

void loop() {
  // Stop the motor
  recvWithStartEndMarkers();
  translateMsg();
  readCommand();
  azStepper.run();
  elStepper.run();
  
}

void readCommand(){
  AccelStepper *targetMotor;
  if(newData == true){
    // Checking target motor
    int targetMotorNumber = atoi(recvTargetMotor);
    if (targetMotorNumber == 1){
       targetMotor = &azStepper;
       targetReduction = azStepperReduction;       
    }
    else if (targetMotorNumber == 2){
       targetMotor = &elStepper;
       targetReduction = elStepperReduction;       
    }
    else{
     Serial.println("Motor does not exist");
    }
    // Checking commands
    if (strcmp(recvCmd,"stop")==0){
     stopMotor(*targetMotor);
     Serial.println("<"+String(targetMotorNumber)+"-ok:stop>");
    }
    else if (strcmp(recvCmd,"move")==0){
     float targetAngle = atof(recvCmdData);
     long targetSteps = angleToSteps(targetAngle);
     moveMotor(*targetMotor, targetSteps);
     Serial.println("<"+String(targetMotorNumber)+"-ok:move-"+String(targetAngle)+">");
    }
    newData = false;  
  }   
}



void moveMotor(AccelStepper &stepper,long targetStep){
  stepper.moveTo(targetStep);
}

long angleToSteps(float angle){
  long steps = round(angle/360*stepsPerLoop*targetReduction);
  return steps;
}

float stepsToAngle(long steps){
  float angle = steps*360.0/stepsPerLoop/targetReduction;
  return angle;
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                receivedLength = ndx;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void translateMsg(){
  // Translates message in the following structure:
  // recvTargetMotor-recvCmd:recvCmdData
  
  if (newData == true){
    char motorLimiter = '-';
    char cmdLimiter = ':';
    boolean reachedMotorLimiter = false;
    boolean reachedCmdLimiter = false;
    for (int i = 0; i < receivedLength; i++){
      if (reachedMotorLimiter == false){
        if (receivedChars[i]== motorLimiter){
            reachedMotorLimiter = true;
            recvTargetMotor[i] = '\0';
            recvTargetMotorLength = i;
        }
        else{
          recvTargetMotor[i] = receivedChars[i];
        }
      }
      else{
        if (reachedCmdLimiter == false){
          if (receivedChars[i]== cmdLimiter){
            reachedCmdLimiter = true;
            recvCmd[i-recvTargetMotorLength-1] = '\0';
            recvCmdLength = i-recvTargetMotorLength-1;
          }
          else{
            recvCmd[i-recvTargetMotorLength-1] = receivedChars[i];
          }
        }
        else{
          recvCmdData[i-recvTargetMotorLength-1-recvCmdLength-1]= receivedChars[i];
          
          if (i == receivedLength - 1){
            recvCmdData[i+1-recvTargetMotorLength-1-recvCmdLength-1] = '\0';
            reachedCmdLimiter = false;
          }
        }
      }
    }
  }
  
}

