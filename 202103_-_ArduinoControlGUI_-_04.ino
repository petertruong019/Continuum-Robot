//----------------------------------------------- Version Update Notes --------------------------------------------------------------
// Revision 1: Only controls the stepper motors, no encoder or any other sensors
// Revision 2: Can run two motors simultaneously. Encoder Feedback implemented. Still an Open loop. No other sensors implemented.
// Revision 3: Encoder driven positioning. Checks the position of the encoder with the current position of the stepper motor.
// Revision 4: Switch to DC motors.
//---------------------------------------------------- HARDWARE PIN VARIABLES -------------------------------------------------------


#define encoderA1 2 //interrupt pin
#define encoderB1 4
#define encoderX1 6
#define encoderA2 3 //interrupt pin
#define encoderB2 5
#define encoderX2 7

#define enA 44 //DC motor 1 Speed PWM Pin
#define enB 45 //DC motor 2 Speed PWM Pin
#define in1 35 //DC motor 1
#define in2 34 //DC motor 1
#define in3 33 //DC motor 2
#define in4 32 //DC motor 2

// ------------------------------------------------- GLOBAL VARIABLES ---------------------------------------------------------------
int InitialEncoderPosition1 = 0;
int InitialEncoderPosition2 = 0;
int CurrentEncoderPosition1 = 0;
int CurrentEncoderPosition2 = 0;

boolean ReceivingSerialMessage = false;
boolean NewData = false;
char startMarker = '<';
char endMarker = '>';
const byte numChars = 32;
char ReceivedChars[numChars];
char rc;
static byte index = 0;

boolean MotorPositionCondition1 = false;
boolean MotorPositionCondition2 = false;
boolean MotorPositionCondition3 = false;
boolean MotorPositionCondition4 = false;
boolean MotorSpeedCondition1 = false;
boolean MotorSpeedCondition2 = false;
boolean MotorSpeedCondition3 = false;
boolean MotorSpeedCondition4 = false;
// ----------------------------------------------- DC MOTOR-RELATED VARIABLES --------------------------------------------------------
float DCMotorPosition1 = 0.0;
float DCMotorPosition2 = 0.0;

float DCMotorSpeed1 = 0.0;
float DCMotorSpeed2 = 0.0;

float DesiredDCMotorPosition1 = 0.0;
float DesiredDCMotorPosition2 = 0.0;

String StrDCMotorPosition1;
String StrDCMotorPosition2;

String StrDCMotorSpeed1;
String StrDCMotorSpeed2;
// -----------------------------------------------------------------------------------------------------------------------------------
void setup() 
{
  // OPEN AND SET THE BAUD RATE OF THE SERIAL PORT
  Serial.begin(9600); 
  pinMode(encoderA1, INPUT);
  pinMode(encoderB1, INPUT);
  pinMode(encoderX1, INPUT);
  pinMode(encoderA2, INPUT);
  pinMode(encoderB2, INPUT);
  pinMode(encoderX2, INPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(enA,OUTPUT);
  pinMode(enB,OUTPUT);

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
  F_RunDCMotors();
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
            StrDCMotorPosition1 = ReceivedChars;
            StrDCMotorPosition1.remove(0,2);
            DCMotorPosition1 = StrDCMotorPosition1.toInt();
            MotorPositionCondition1 = true;
          }
          else if (ReceivedChars[0] == 'S' && ReceivedChars[1] == '1')
          {
            StrDCMotorSpeed1 = ReceivedChars;
            StrDCMotorSpeed1.remove(0,2);
            DCMotorSpeed1 = StrDCMotorSpeed1.toInt();
            MotorSpeedCondition1 = true;
          }
          else if (ReceivedChars[0] == 'P' && ReceivedChars[1] == '2')
          {
            StrDCMotorPosition2 = ReceivedChars;
            StrDCMotorPosition2.remove(0,2);
            DCMotorPosition2 = StrDCMotorPosition2.toInt();
            MotorPositionCondition2 = true;
          }
          else if (ReceivedChars[0] == 'S' && ReceivedChars[1] == '2')
          {
            StrDCMotorSpeed2 = ReceivedChars;
            StrDCMotorSpeed2.remove(0,2);
            DCMotorSpeed2 = StrDCMotorSpeed2.toInt();
            MotorSpeedCondition2 = true;
          }
          
}

//----------------------------------------------- DC MOTOR FUNCTIONS ---------------------------------------------
void F_DCMotor1_CW()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
void F_DCMotor1_CCW()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void F_DCMotor2_CW()
{
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}
void F_DCMotor2_CCW()
{
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
void F_DCMotor1_Speed()
{
  analogWrite(enA,DCMotorSpeed1); //Allowable speed value range: 0-255
}
void F_DCMotor2_Speed()
{
  analogWrite(enB,DCMotorSpeed2); //Allowable speed value range: 0-255
}

void F_RunDCMotors()
{
  if (MotorPositionCondition1 == true && MotorSpeedCondition1 == true && MotorPositionCondition2 == true && MotorSpeedCondition2 == true)
    {
      F_DCMotor2_Speed(); //Assigns the speed of DC Motor 1
    
      while (CurrentEncoderPosition2 != DCMotorPosition2)
        {
          F_ReadEncoderPosition();
          F_DCMotor2_CCW();
        }
      analogWrite(enB,0);
      MotorPositionCondition1 = false;
      MotorSpeedCondition1 = false;
      MotorPositionCondition2 = false;
      MotorSpeedCondition2 = false;

    }
}
//----------------------------------------------- ENCODER FUNCTIONS ----------------------------------------------
void F_ReadEncoderPosition()
{
  if (InitialEncoderPosition1  != CurrentEncoderPosition1 /*|| InitialEncoderPosition2 != CurrentEncoderPosition2*/)
    {
      CurrentEncoderPosition1 = InitialEncoderPosition1;
      //CurrentEncoderPosition2 = InitialEncoderPosition2;
      Serial.println(CurrentEncoderPosition1); //Serial.print(","); Serial.println(CurrentEncoderPosition2);
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
