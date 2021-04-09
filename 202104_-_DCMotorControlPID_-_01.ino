
#define IN1 33 //DC motor 2
#define IN2 32 //DC motor 2
#define PWM1 45

#define ENCA 2 //interrupt pin
#define ENCB 4 

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

boolean ReceivingSerialMessage = false;
boolean NewData = false;
char startMarker = '<';
char endMarker = '>';
const byte numChars = 32;
char ReceivedChars[numChars];
char rc;
static byte index = 0;

// PID constants


// ----------------------------------------------- DC MOTOR-RELATED VARIABLES --------------------------------------------------------
float DCMotorPosition1 = 0.0;
float DCMotorPosition2 = 0.0;

float DCMotorSpeed1 = 0.0;
float DCMotorSpeed2 = 0.0;

String StrDCMotorPosition1;
String StrDCMotorPosition2;

String StrDCMotorSpeed1;
String StrDCMotorSpeed2;

boolean MotorPositionCondition1 = false;
boolean MotorPositionCondition2 = false;
boolean MotorPositionCondition3 = false;
boolean MotorPositionCondition4 = false;
boolean MotorSpeedCondition1 = false;
boolean MotorSpeedCondition2 = false;
boolean MotorSpeedCondition3 = false;
boolean MotorSpeedCondition4 = false;
// -----------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB),readEncoder,RISING);
  Serial.println("target pos");
}
// PID Constants for Geartisan Motor (200rpm)
/* speed = 100
 * Sine-wave: 250*sin(prevT/1e6)
 * kp = 3
 * kd = .97
 * ki = .695
 * 
 * target = 200
 * kp = .58
 * kd = 0
 * ki = 0
 */

// PID Constants for Worm Gear Motor (10rpm)
/*
 * speed 255
 * target = 1000
 * kp = .5695
 * ki = 0
 * kd = 0
 */
void loop() 
{
// set target position
  int target = 1000;
  //int target = 750*sin(prevT/1e6);
  
  float kp = 6.55; 
  float kd = 2.25;
  float ki = 1.75;
  
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
  
  // error
  int e = pos-target;
  
  // derivative
  float dedt = (e-eprev)/(deltaT);
  
  // integral
  eintegral = eintegral + e*deltaT;
  
  // control signal equation
  float u = kp*e + kd*dedt + ki*eintegral;
  
  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }
  // signal the motor
  setMotor(dir,pwr,PWM1,IN1,IN2);

  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
  
  
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}
void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    pos++;
  }
  else{
    pos--;
  }
}

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
          /*if (ReceivedChars[0] == 'P' && ReceivedChars[1] == '1')
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
          }*/
          
}
