//----------------------------------------------- Version Update Notes --------------------------------------------------------------
// Revision 1: Only controls the stepper motors, no encoder or any other sensors
// Revision 2: Can run two motors simultaneously. Encoder Feedback implemented. Still an Open loop.

//---------------------------------------------------- LIBRARIES --------------------------------------------------------------------
#include <AccelStepper.h>
//---------------------------------------------------- HARDWARE PIN VARIABLES -------------------------------------------------------
#define stepper1_dir_pin 26
#define stepper1_step_pin 27
#define stepper1_COM_pin 28
#define stepper2_dir_pin 29
#define stepper2_step_pin 30
#define stepper2_COM_pin 31 

#define encoderA1 2 //interrupt pin
#define encoderB1 4
#define encoderX1 6
#define encoderA2 3 //interrupt pin
#define encoderB2 5
#define encoderX2 7

#define force_sensor1 A13
#define force_sensor2 A14
#define force_sensor3 A15

// --------- DEFINE STEPPER MOTORS IN THE ACCELSTEPPER LIBRARY - (BIPOLAR,STEP_PIN,DIR_PIN)------------------------------------------
AccelStepper stepper1 (1, stepper1_step_pin, stepper1_dir_pin); 
AccelStepper stepper2 (1, stepper2_step_pin, stepper2_dir_pin); 

// ------------------------------------------------- GLOBAL VARIABLES ---------------------------------------------------------------
int InitialEncoderPosition1 = 0;
int InitialEncoderPosition2 = 0;
int CurrentEncoderPosition1 = 0;
int CurrentEncoderPosition2 = 0;

int read_force_sensor1;
int read_force_sensor2;
int read_force_sensor3;

boolean ReceivingSerialMessage = false;
boolean NewData = false;
char startMarker = '<';
char endMarker = '>';
const byte numChars = 32;
char ReceivedChars[numChars];
char rc;
static byte index = 0;


// ----------------------------------------------- STEPPER-RELATED VARIABLES --------------------------------------------------------
float StepsPerRev = 200.0; //change according to stepper motor being used. Check stepper motor datasheet
float StepperMotorPosition1 = 0.0;
float StepperMotorPosition2 = 0.0;
float StepperMotorSpeed1 = 0.0;
float StepperMotorSpeed2 = 0.0;
float StepperMotorSpeed3 = 0.0;
float StepperMotorSpeed4 = 0.0;

String StrStepperMotorPosition1;
String StrStepperMotorPosition2;
String StrStepperMotorPosition3;
String StrStepperMotorPosition4;

String StrStepperMotorSpeed1;
String StrStepperMotorSpeed2;
String StrStepperMotorSpeed3;
String StrStepperMotorSpeed4;

boolean MotorPositionCondition1 = false;
boolean MotorPositionCondition2 = false;
boolean MotorPositionCondition3 = false;
boolean MotorPositionCondition4 = false;
boolean MotorSpeedCondition1 = false;
boolean MotorSpeedCondition2 = false;
boolean MotorSpeedCondition3 = false;
boolean MotorSpeedCondition4 = false;
// -----------------------------------------------------------------------------------------------------------------------------------
void setup() 
{
  // OPEN AND SET THE BAUD RATE OF THE SERIAL PORT
  Serial.begin(115200); 
  
  // INITIALIZE STEPPER MOTOR PARAMETERS HERE
  stepper1.setMaxSpeed(200); 
  stepper2.setMaxSpeed(200);
  stepper1.setAcceleration(1500);
  stepper2.setAcceleration(1500);
  pinMode(encoderA1, INPUT);
  pinMode(encoderB1, INPUT);
  pinMode(encoderX1, INPUT);
  pinMode(encoderA2, INPUT);
  pinMode(encoderB2, INPUT);
  pinMode(encoderX2, INPUT);
  pinMode(stepper1_COM_pin, OUTPUT);
  pinMode(stepper2_COM_pin, OUTPUT);
  pinMode(force_sensor1, INPUT);
  pinMode(force_sensor2, INPUT);
  pinMode(force_sensor3, INPUT);
  digitalWrite(stepper1_COM_pin, HIGH);
  digitalWrite(stepper2_COM_pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoderA1),F_Encoder1,FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderA2),F_Encoder2,FALLING);
  
}

void loop() 
{
  int CurrentEncoderPosition1 = 0;
  int CurrentEncoderPosition2 = 0;
  
  while (Serial.available()>0 && NewData == false) //Checks to see if anything is in the serial buffer
    {
      F_ReceiveSerialMessage();
    }
  F_PrintReceiveSerialMessage();
  F_RunStepperMotorMotion();
  
}

//------------------------------------------------------ FUNCTIONS ----------------------------------------------------
void F_ReceiveSerialMessage()
{
    rc = Serial.read();
    if (rc == startMarker)
      {
        ReceivingSerialMessage = true;
      }
    else if (ReceivingSerialMessage == true)
      {
        if (rc != endMarker)
          {
            ReceivedChars[index] = rc;
            index++;
            //Serial.print(index); Serial.print(" ");
            //Serial.println(ReceivedChars);
            if (index >= numChars)
              {
                index = numChars - 1;
              }
          }
        else
          {
            ReceivedChars[index] = '\0'; //this terminates the string
            ReceivingSerialMessage = false;
            index = 0;
            NewData = true;
          }
      }
    
}
void F_PrintReceiveSerialMessage()
{
  if (NewData == true)
      {
          F_CheckSerialProtocol();
          NewData = false;
          
      }
  
}

void F_CheckSerialProtocol()
{
          if (ReceivedChars[0] == 'P' && ReceivedChars[1] == '1')
          {
            StrStepperMotorPosition1 = ReceivedChars;
            StrStepperMotorPosition1.remove(0,2);
            StepperMotorPosition1 = StrStepperMotorPosition1.toInt();
            MotorPositionCondition1 = true;
          }
          if (ReceivedChars[0] == 'S' && ReceivedChars[1] == '1')
          {
            StrStepperMotorSpeed1 = ReceivedChars;
            StrStepperMotorSpeed1.remove(0,2);
            StepperMotorSpeed1 = StrStepperMotorSpeed1.toInt();
            MotorSpeedCondition1 = true;
          }
          if (ReceivedChars[0] == 'P' && ReceivedChars[1] == '2')
          {
            StrStepperMotorPosition2 = ReceivedChars;
            StrStepperMotorPosition2.remove(0,2);
            StepperMotorPosition2 = StrStepperMotorPosition2.toInt();
            MotorPositionCondition2 = true;
          }
          if (ReceivedChars[0] == 'S' && ReceivedChars[1] == '2')
          {
            StrStepperMotorSpeed2 = ReceivedChars;
            StrStepperMotorSpeed2.remove(0,2);
            StepperMotorSpeed2 = StrStepperMotorSpeed2.toInt();
            MotorSpeedCondition2 = true;
          }
}

void F_RunStepperMotorMotion()
{
  if (MotorPositionCondition1 == true && MotorSpeedCondition1 == true && MotorPositionCondition2 == true && MotorSpeedCondition2 == true)
    {
      stepper1.setMaxSpeed(StepperMotorSpeed1); //we assign the speed and position
      stepper1.moveTo(StepperMotorPosition1);
      stepper2.setMaxSpeed(StepperMotorSpeed2); //we assign the speed and position
      stepper2.moveTo(StepperMotorPosition2);
      while(stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0)
        {
          F_ReadEncoderPosition();
          stepper1.run(); //this keeps stepping the motor once until it reach its desired position
          stepper2.run();
        }
      stepper1.setCurrentPosition(0); //resets positin back to zero
      stepper2.setCurrentPosition(0);
      MotorPositionCondition1 = false;
      MotorSpeedCondition1 = false;
      MotorPositionCondition2 = false;
      MotorSpeedCondition2 = false;
    }
}

void F_ReadEncoderPosition()
{
  if (InitialEncoderPosition1  != CurrentEncoderPosition1 || InitialEncoderPosition2 != CurrentEncoderPosition2)
    {
      CurrentEncoderPosition1 = InitialEncoderPosition1;
      CurrentEncoderPosition2 = InitialEncoderPosition2;
      Serial.print(CurrentEncoderPosition1); Serial.print(","); Serial.println(CurrentEncoderPosition2);
    }
}

void F_Encoder1()
{
  if (digitalRead(encoderB1) == LOW) 
  {
    InitialEncoderPosition1++;
  }
  else
  {
    InitialEncoderPosition1--;
  }
}

void F_Encoder2()
{
  if (digitalRead(encoderB2) == LOW) 
  {
    InitialEncoderPosition2++;
  }
  else
  {
    InitialEncoderPosition2--;
  }
}
