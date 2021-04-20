// ---------------------- Revisions --------------------------------------
// Revision 1: Preliminary Version. Code is taken from DCMotorControlPID.ino ver 4. Uses Jevois camera feedback instead of encoders.
//            
// ----------------------------------------------------------------------

#define IN1 35 //DC motor 1
#define IN2 34 //DC motor 1
#define PWM1 7 //DC motor 1 speed

#define IN3 33 //DC motor 2
#define IN4 32 //DC motor 2
#define PWM2 6 // DC motor 2 speed

#define ENCA1 2 //interrupt pin
#define ENCB1 4 
#define ENCA2 3 //interrupt pin
#define ENCB2 5 


int pos1 = 0; // This would be the y axis pixels
int pos2 = 0; // This would be the x axis pixels
long prevTime1 = 0;
long prevTime2 = 0;
float eprev1 = 0;
float eprev2 = 0;
float eintegral1 = 0;
float eintegral2 = 0;

boolean ReceivingSerialMessage = false;
boolean NewData = false;
char startMarker = '<';
char endMarker = '>';
const byte numChars = 32;
char ReceivedChars[numChars];
char rc;
static byte index = 0;

// ----------------------------------------------- DC MOTOR-RELATED VARIABLES --------------------------------------------------------
float DCMotorPosition1 = 0.0;
float DCMotorPosition2 = 0.0;
float JevoisXaxis = 0.0;
float JevoisYaxis = 0.0;

String StrDCMotorPosition1;
String StrDCMotorPosition2;
String StrJevoisXaxis;
String StrJevoisYaxis;

boolean MotorPositionCondition1 = false;
boolean MotorPositionCondition2 = false;
boolean JevoisConditionX = false;
boolean JevoisConditionY = false;
// -----------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  TCCR4B = TCCR4B & B11111000 | B00000001; // This sets the PWM frequency to 32khz 
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB1),readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2),readEncoder2,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB2),readEncoder2,RISING);
  //Serial.println("target pos");
}

 
void loop() 
{

  while(Serial.available()>0 && NewData == false)
    {
      F_ReceiveSerialMessage();
    }
   
  F_CheckSerialProtocol();
  F_RunJevoisPID();
  
}
// ----------------------------------------------- JEVOIS CAMERA FUNCTIONS -----------------------------------------------------------------
void F_RunJevoisPID()
{
  if (JevoisConditionX == true && JevoisConditionY == true)
    {
      // set target position
      int target1 = JevoisYaxis;
      int target2 = JevoisXaxis;
  
      //PID GAINS FOR MOTOR 1
      float kp1 = 167.5;
      float kd1 = 1.25;
      float ki1 = 0;
  
      //PID GAINS FOR MOTOR 2 
      float kp2 = 167.5;
      float kd2 = 1.25;
      float ki2 = 0;
  
      // TIME DIFFERENCE -----------------------------------------------
      long currTime1 = micros();
      long currTime2 = micros();
      float deltaT1 = ((float) (currTime1 - prevTime1))/( 1.0e6 );
      float deltaT2 = ((float) (currTime2 - prevTime2))/( 1.0e6 );
      prevTime1 = currTime1;
      prevTime2 = currTime2;
      // ERROR ---------------------------------------------------------
      int e1 = pos1-target1;  
      int e2 = pos2-target2; 
      // DERIVATIVE ----------------------------------------------------
      float dedt1 = (e1-eprev1)/(deltaT1);
      float dedt2 = (e2-eprev2)/(deltaT2);
      // INTEGRAL ------------------------------------------------------
      eintegral1 = eintegral1 + e1*deltaT1;
      eintegral2 = eintegral2 + e2*deltaT2;
      // CONTROL SIGNAL EQUATION ---------------------------------------
      float u1 = kp1*e1 + kd1*dedt1 + ki1*eintegral1;
      float u2 = kp2*e2 + kd2*dedt2 + ki2*eintegral2;
      // MOTOR SPEED ---------------------------------------------------
      float pwr1 = fabs(u1); //fab takes a single argument (in double) and returns the absolute value of that number.
      float pwr2 = fabs(u2);
      if( pwr1 > 255 )
      {
        pwr1 = 255;
      }
      if( pwr2 > 255 )
      {
        pwr2 = 255;
      }
      // MOTOR DIRECTION -----------------------------------------------
      int dir1 = 1;
      int dir2 = 1;
      if(u1<0) //if the control signal is negative, change the direction of the motors
      {
        dir1 = -1;
      }
      if(u2<0)
      {
        dir2 = -1;
      }
      // ASSIGN SIGNAL TO MOTORS ---------------------------------------
      setMotor1(dir1,pwr1,PWM1,IN1,IN2);
      setMotor2(dir2,pwr2,PWM2,IN3,IN4);
      // STORE PREVIOUS ERROR ------------------------------------------
      eprev1 = e1;
      eprev2 = e2;
  
      Serial.print(target1);Serial.print(" ");Serial.print(target2);Serial.print(" ");Serial.print(pos1);Serial.print(" ");Serial.print(pos2);Serial.println();
    }
}
//------------------------------------------------ DC MOTOR PID CONTROL FUNCTIONS ----------------------------------------------------------
void F_RunDCPID()
{
  if (MotorPositionCondition1 == true && MotorPositionCondition2 == true)
  {
  // set target position
  int target1 = DCMotorPosition1;
  int target2 = DCMotorPosition2;
  
  //PID GAINS FOR MOTOR 1
  float kp1 = 167.5;
  float kd1 = 1.25;
  float ki1 = 0;
  
  //PID GAINS FOR MOTOR 2 
  float kp2 = 167.5;
  float kd2 = 1.25;
  float ki2 = 0;
  
  // TIME DIFFERENCE -----------------------------------------------
  long currTime1 = micros();
  long currTime2 = micros();
  float deltaT1 = ((float) (currTime1 - prevTime1))/( 1.0e6 );
  float deltaT2 = ((float) (currTime2 - prevTime2))/( 1.0e6 );
  prevTime1 = currTime1;
  prevTime2 = currTime2;
  // ERROR ---------------------------------------------------------
  int e1 = pos1-target1;  
  int e2 = pos2-target2; 
  // DERIVATIVE ----------------------------------------------------
  float dedt1 = (e1-eprev1)/(deltaT1);
  float dedt2 = (e2-eprev2)/(deltaT2);
  // INTEGRAL ------------------------------------------------------
  eintegral1 = eintegral1 + e1*deltaT1;
  eintegral2 = eintegral2 + e2*deltaT2;
  // CONTROL SIGNAL EQUATION ---------------------------------------
  float u1 = kp1*e1 + kd1*dedt1 + ki1*eintegral1;
  float u2 = kp2*e2 + kd2*dedt2 + ki2*eintegral2;
  // MOTOR SPEED ---------------------------------------------------
  float pwr1 = fabs(u1); //fab takes a single argument (in double) and returns the absolute value of that number.
  float pwr2 = fabs(u2);
  if( pwr1 > 255 ){
    pwr1 = 255;
  }
  if( pwr2 > 255 ){
    pwr2 = 255;
  }
  // MOTOR DIRECTION -----------------------------------------------
  int dir1 = 1;
  int dir2 = 1;
  if(u1<0) //if the control signal is negative, change the direction of the motors
  {
    dir1 = -1;
  }
  if(u2<0)
  {
    dir2 = -1;
  }
  // ASSIGN SIGNAL TO MOTORS ---------------------------------------
  setMotor1(dir1,pwr1,PWM1,IN1,IN2);
  setMotor2(dir2,pwr2,PWM2,IN3,IN4);
  // STORE PREVIOUS ERROR ------------------------------------------
  eprev1 = e1;
  eprev2 = e2;
  
  Serial.print(target1);Serial.print(" ");Serial.print(target2);Serial.print(" ");Serial.print(pos1);Serial.print(" ");Serial.print(pos2);Serial.println();
  }
  
}

void setMotor1(int dir1, int pwmVal1, int pwm1, int in1, int in2){
  analogWrite(pwm1,pwmVal1);
  if(dir1 == 1)
  {
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir1 == -1)
  {
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void setMotor2(int dir2, int pwmVal2, int pwm2, int in3, int in4){
  analogWrite(pwm2,pwmVal2);
  if(dir2 == 1)
  {
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
  }
  else if(dir2 == -1)
  {
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
  }
  else
  {
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
  }  
}

void readEncoder1()
{
  int b1 = digitalRead(ENCB1);
  if(b1 > 0)
  {
    pos1++;
  }
  else
  {
    pos1--;
  }
}
void readEncoder2()
{
  int b2 = digitalRead(ENCB2);
  if(b2 > 0)
  {
    pos2++;
  }
  else
  {
    pos2--;
  }
}


// ------------------------------------------ SERIAL FUNCTIONS ------------------------------------------------------
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
void F_CheckSerialProtocol()
{
  if (NewData == true)
      {
          if (ReceivedChars[0] == '1' && ReceivedChars[1] == 'p')
          {
            StrDCMotorPosition1 = ReceivedChars;
            StrDCMotorPosition1.remove(0,2);
            DCMotorPosition1 = StrDCMotorPosition1.toInt();
            MotorPositionCondition1 = true;
          }
          else if (ReceivedChars[0] == '2' && ReceivedChars[1] == 'p')
          {
            StrDCMotorPosition2 = ReceivedChars;
            StrDCMotorPosition2.remove(0,2);
            DCMotorPosition2 = StrDCMotorPosition2.toInt();
            MotorPositionCondition2 = true;
          }
          else if (ReceivedChars[0] == 'x')
          {
            StrJevoisXaxis = ReceivedChars;
            StrJevoisXaxis.remove(0,1);
            JevoisXaxis = StrJevoisXaxis.toInt();
            JevoisConditionX = true;
          }
          else if (ReceivedChars[0] == 'y')
          {
            StrJevoisYaxis = ReceivedChars;
            StrJevoisYaxis.remove(0,1);
            JevoisYaxis = StrJevoisYaxis.toInt();
            JevoisConditionY = true;
          }
          NewData = false;
      }
  
}
