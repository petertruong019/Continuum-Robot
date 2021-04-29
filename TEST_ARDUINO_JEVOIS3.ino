// ---------------------- Revisions --------------------------------------
// Revision 1: Preliminary Version. Code is taken from DCMotorControlPID.ino ver 4. Uses Jevois camera feedback instead of encoders.
// Revision 2: Added joystick manual control loop.
// Revision 3: Added circle pattern PID.
// ----------------------------------------------------------------------
//#include <SoftwareSerial.h>
//SoftwareSerial cam(0,1);
//#define SERIAL Serial1

#define IN1 35 //DC motor 1
#define IN2 34 //DC motor 1
#define PWM1 9 //DC motor 1 speed

#define IN3 33 //DC motor 2
#define IN4 32 //DC motor 2
#define PWM2 6 // DC motor 2 speed

#define ENCA1 2 //interrupt pin
#define ENCB1 4
#define ENCA2 3 //interrupt pin
#define ENCB2 5

#define JOYSTICK_X A0
#define JOYSTICK_Y A1
#define JOYSTICK_SW 25

int pos1 = 0; // This would be the y axis
int pos2 = 0; // This would be the x axis
long prevTime1 = 0;
long prevTime2 = 0;
float eprev1 = 0;
float eprev2 = 0;
float pi = 3.14;
float e1 = 0;
float e2 = 0;
int target1 = 0;
int target2 = 0;
float eintegral1 = 0;
float eintegral2 = 0;
int JoyPosition_X = 0;
int JoyPosition_Y = 0;
int dir1 = 0;
int dir2 = 0;

boolean ReceivingSerialMessage = false;
boolean NewData = false;
boolean newData = false;
char startMarker = '<';
char endMarker = '>';
const byte numChars = 32;
char ReceivedChars[numChars];
char receivedChars[numChars];
static byte index = 0;
char rc;
char jrc;

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

boolean CircleMove = false;

// -----------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  TCCR4B = TCCR4B & B11111000 | B00000001; // This sets the PWM frequency to 32khz
  TCCR2B = TCCR2B & B11111000 | B00000001;
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
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_SW, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB1), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB2), readEncoder2, RISING);
  //Serial.println("target pos");
}

// ------------------------------------------------------------- MAIN LOOP -------------------------------------------------
void loop() {
  //ReceiveSerialJevois();
  //ReceiveSerialArduino();
  //ShowJevoisXY();
  //F_CheckSerialProtocol();
  //F_RunJevoisArduino();
  //F_RunDCPID();
  F_RunJoyStick();
  //F_RunCirclePID();
  //newData = false;
}

// ----------------------------------------------- JEVOIS CAMERA FUNCTIONS FOR ARDUINO INTERFACE -----------------------------------------------------------------
void F_RunJevoisArduino()
{
  if (JevoisConditionX == true && JevoisConditionY == true)
  {
    // set target position
    int target1 = 0;
    int target2 = 0;

    //PID GAINS FOR MOTOR 1
    float kp1 = 40; //19/20;
    float kd1 = 0.75; //2;
    float ki1 = 0; //0.002/.0018;

    //PID GAINS FOR MOTOR 2
    float kp2 = 40;//19/20;
    float kd2 = 0.75;//2;
    float ki2 = 0;//0.002/.0018;

    //CONTROLLER BIAS FOR MOTOR 1
    float ub1 = 0;

    //CONTROLLER BIAS FOR MOTOR 2
    float ub2 = 0;

    // TIME DIFFERENCE -----------------------------------------------
    long currTime1 = micros();
    long currTime2 = micros();
    float deltaT1 = ((float) (currTime1 - prevTime1)) / ( 1.0e6 );
    float deltaT2 = ((float) (currTime2 - prevTime2)) / ( 1.0e6 );
    prevTime1 = currTime1;
    prevTime2 = currTime2;
    // ERROR ---------------------------------------------------------
    int e1 = JevoisYaxis - target1;
    int e2 = JevoisXaxis - target2;

    //Serial.print(JevoisYaxis); Serial.println(JevoisXaxis);
    // DERIVATIVE ----------------------------------------------------
    float dedt1 = (e1 - eprev1) / (deltaT1);
    float dedt2 = (e2 - eprev2) / (deltaT2);
    // INTEGRAL ------------------------------------------------------
    eintegral1 = eintegral1 + e1 * deltaT1;
    eintegral2 = eintegral2 + e2 * deltaT2;
    // CONTROL SIGNAL EQUATION ---------------------------------------
    float u1 = kp1 * e1 + kd1 * dedt1 + ki1 * eintegral1 + ub1;
    float u2 = kp2 * e2 + kd2 * dedt2 + ki2 * eintegral2 + ub2;
    // MOTOR SPEED ---------------------------------------------------
    float pwr1 = fabs(u1); //fab takes a single argument (in double) and returns the absolute value of that number.
    float pwr2 = fabs(u2);
    if ( pwr1 > 255 )
    {
      pwr1 = 255;
    }
    if ( pwr2 > 255 )
    {
      pwr2 = 255;
    }

    // MOTOR DIRECTION -----------------------------------------------
    int dir1 = 1;
    int dir2 = 1;
    if (u1 < 0)
    {
      dir1 = -1;
    }
    if (u2 < 0)
    {
      dir2 = -1;
    }
    //      if(fabs(e1) <= 5)
    //      {
    //        dir1 = 0;
    //      }
    //      if (fabs(e2) <= 5)
    //      {
    //        dir2 = 0;
    //      }
    // ASSIGN SIGNAL TO MOTORS ---------------------------------------
    setMotor1(dir1, pwr1, PWM1, IN1, IN2);
    setMotor2(dir2, pwr2, PWM2, IN3, IN4);
    // STORE PREVIOUS ERROR ------------------------------------------
    eprev1 = e1;
    eprev2 = e2;
    Serial.print(target1); Serial.print(" "); Serial.print(target2); Serial.print(" "); Serial.print(JevoisYaxis); Serial.print(" "); Serial.print(JevoisXaxis); Serial.println();
    //Serial.print("Time Elapsed: "); Serial.println(deltaT1);
  }
}

//------------------------------------------------ DC MOTOR PID CONTROL FUNCTIONS ----------------------------------------------------------
void F_RunCirclePID()
{
  // x^2 + y^2 = r^2 equation of circle
  // r = # of rotary encoder pulses in one direction (max = 6250)
  // y = sqrt(r^2 - x^2)
  //PID GAINS FOR MOTOR 1
  float kp1 = 167.5;
  float kd1 = 1.25;
  float ki1 = 0;

  //PID GAINS FOR MOTOR 2
  float kp2 = 167.5;
  float kd2 = 1.25;
  float ki2 = 0;
  int cy;
  int cr = 4000;

for (int cdeg = 0; cdeg <= 360; cdeg = cdeg + 3)
  {
    float crad = cdeg*(pi/180);
    double cy = sin(crad);
    double cx = cos(crad);
    int target1 = cy*cr;
    int target2 = cx*cr;
    CircleMove = true;
    while (CircleMove == true)
    {
      // TIME DIFFERENCE -----------------------------------------------
      long currTime1 = micros();
      long currTime2 = micros();
      float deltaT1 = ((float) (currTime1 - prevTime1)) / ( 1.0e6 );
      float deltaT2 = ((float) (currTime2 - prevTime2)) / ( 1.0e6 );
      prevTime1 = currTime1;
      prevTime2 = currTime2;
      // ERROR ---------------------------------------------------------
      int e1 = pos1 - target1;
      int e2 = pos2 - target2;
      // DERIVATIVE ----------------------------------------------------
      float dedt1 = (e1 - eprev1) / (deltaT1);
      float dedt2 = (e2 - eprev2) / (deltaT2);
      // INTEGRAL ------------------------------------------------------
      eintegral1 = eintegral1 + e1 * deltaT1;
      eintegral2 = eintegral2 + e2 * deltaT2;
      // CONTROL SIGNAL EQUATION ---------------------------------------
      float u1 = kp1 * e1 + kd1 * dedt1 + ki1 * eintegral1;
      float u2 = kp2 * e2 + kd2 * dedt2 + ki2 * eintegral2;
      // MOTOR SPEED ---------------------------------------------------
      float pwr1 = fabs(u1); //fab takes a single argument (in double) and returns the absolute value of that number.
      float pwr2 = fabs(u2);
      if ( pwr1 > 255 ) {
        pwr1 = 255;
      }
      if ( pwr2 > 255 ) {
        pwr2 = 255;
      }
      // MOTOR DIRECTION -----------------------------------------------
      int dir1 = 1;
      int dir2 = 1;
      if (u1 < 0) //if the control signal is negative, change the direction of the motors
      {
        dir1 = -1;
      }
      if (u2 < 0)
      {
        dir2 = -1;
      }
      // ASSIGN SIGNAL TO MOTORS ---------------------------------------
      setMotor1(dir1, pwr1, PWM1, IN1, IN2);
      setMotor2(dir2, pwr2, PWM2, IN3, IN4);
      // STORE PREVIOUS ERROR ------------------------------------------
      eprev1 = e1;
      eprev2 = e2;
      if ( fabs(e1) <= 5 && fabs(e2) <= 5)
      {
        CircleMove = false;
      }
      Serial.print(target1); Serial.print(" "); Serial.print(target2); Serial.print(" "); Serial.print(pos1); Serial.print(" "); Serial.print(pos2); Serial.println();
    }
  }
}


// ----------------------------------------------------------------------- JOYSTICK MANUAL CONTROL ------------------------------------------------
void F_RunJoyStick()
{
  JoyPosition_X = analogRead(JOYSTICK_X);
  JoyPosition_Y = analogRead(JOYSTICK_Y);

  int pwr1 = map(JoyPosition_X, 0, 1023, -255, 255);
  int pwr2 = map(JoyPosition_Y, 0, 1023, -255, 255);

  // MOTOR DIRECTION -----------------------------------------------
  int dir1 = 1;
  int dir2 = 1;

  if (JoyPosition_X < 512)
  {
    dir1 = -1;
    pwr1 = -1 * pwr1;
  }

  if (JoyPosition_Y < 512)
  {
    dir2 = -1;
    pwr2 = -1 * pwr2;
  }

  setMotor1(dir1, pwr1, PWM1, IN1, IN2);
  setMotor2(dir2, pwr2, PWM2, IN3, IN4);
  Serial.print("X: ");
  Serial.print(JoyPosition_X); Serial.print(" "); Serial.print(pwr1); Serial.print(" "); Serial.print(dir1); Serial.print(" ");
  Serial.print("Y: ");
  Serial.print(JoyPosition_Y); Serial.print(" "); Serial.print(pwr2); Serial.print(" "); Serial.println(dir2);
}


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
    float deltaT1 = ((float) (currTime1 - prevTime1)) / ( 1.0e6 );
    float deltaT2 = ((float) (currTime2 - prevTime2)) / ( 1.0e6 );
    prevTime1 = currTime1;
    prevTime2 = currTime2;
    // ERROR ---------------------------------------------------------
    int e1 = pos1 - target1;
    int e2 = pos2 - target2;
    // DERIVATIVE ----------------------------------------------------
    float dedt1 = (e1 - eprev1) / (deltaT1);
    float dedt2 = (e2 - eprev2) / (deltaT2);
    // INTEGRAL ------------------------------------------------------
    eintegral1 = eintegral1 + e1 * deltaT1;
    eintegral2 = eintegral2 + e2 * deltaT2;
    // CONTROL SIGNAL EQUATION ---------------------------------------
    float u1 = kp1 * e1 + kd1 * dedt1 + ki1 * eintegral1;
    float u2 = kp2 * e2 + kd2 * dedt2 + ki2 * eintegral2;
    // MOTOR SPEED ---------------------------------------------------
    float pwr1 = fabs(u1); //fab takes a single argument (in double) and returns the absolute value of that number.
    float pwr2 = fabs(u2);
    if ( pwr1 > 255 ) {
      pwr1 = 255;
    }
    if ( pwr2 > 255 ) {
      pwr2 = 255;
    }

    // MOTOR DIRECTION -----------------------------------------------
    int dir1 = 1;
    int dir2 = 1;
    if (u1 < 0) //if the control signal is negative, change the direction of the motors
    {
      dir1 = -1;
    }
    if (u2 < 0)
    {
      dir2 = -1;
    }

    // ASSIGN SIGNAL TO MOTORS ---------------------------------------
    setMotor1(dir1, pwr1, PWM1, IN1, IN2);
    setMotor2(dir2, pwr2, PWM2, IN3, IN4);
    // STORE PREVIOUS ERROR ------------------------------------------
    eprev1 = e1;
    eprev2 = e2;

    Serial.print(target1); Serial.print(" "); Serial.print(target2); Serial.print(" "); Serial.print(pos1); Serial.print(" "); Serial.print(pos2); Serial.print(" ");
    Serial.print(pwr1); Serial.print(" "); Serial.println(pwr2);
  }

}

void setMotor1(int dir1, int pwmVal1, int pwm1, int in1, int in2) {
  analogWrite(pwm1, pwmVal1);
  if (dir1 == 1)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir1 == -1)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void setMotor2(int dir2, int pwmVal2, int pwm2, int in3, int in4) {
  analogWrite(pwm2, pwmVal2);
  if (dir2 == 1)
  {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if (dir2 == -1)
  {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else
  {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
}

void readEncoder1()
{
  int b1 = digitalRead(ENCB1);
  if (b1 > 0)
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
  if (b2 > 0)
  {
    pos2++;
  }
  else
  {
    pos2--;
  }
}


// ------------------------------------------ SERIAL FUNCTIONS ------------------------------------------------------
void ReceiveSerialJevois() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char jrc;

  while (Serial1.available() > 0 && newData == false) {
    jrc = Serial1.read();

    if (recvInProgress == true) {
      if (jrc != endMarker) {
        receivedChars[ndx] = jrc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (jrc == startMarker) {
      recvInProgress = true;
    }
  }
}

void ReceiveSerialArduino() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
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
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void ShowJevoisXY() {
  if (newData == true) {
    Serial.println(receivedChars);
  }
}

void F_CheckSerialProtocol()
{
  if (newData == true)
  {
    if (receivedChars[0] == '1' && receivedChars[1] == 'p')
    {
      StrDCMotorPosition1 = receivedChars;
      StrDCMotorPosition1.remove(0, 2);
      DCMotorPosition1 = StrDCMotorPosition1.toInt();
      MotorPositionCondition1 = true;
    }
    else if (receivedChars[0] == '2' && receivedChars[1] == 'p')
    {
      StrDCMotorPosition2 = receivedChars;
      StrDCMotorPosition2.remove(0, 2);
      DCMotorPosition2 = StrDCMotorPosition2.toInt();
      MotorPositionCondition2 = true;
    }
    else if (receivedChars[0] == 'J' && receivedChars[1] == 'x')
    {
      StrJevoisXaxis = receivedChars;
      StrJevoisXaxis.remove(0, 2);
      JevoisXaxis = StrJevoisXaxis.toInt();
      //Serial.print(JevoisXaxis);
      JevoisConditionX = true;
    }
    else if (receivedChars[0] == 'J' && receivedChars[1] == 'y')
    {
      StrJevoisYaxis = receivedChars;
      StrJevoisYaxis.remove(0, 2);
      JevoisYaxis = StrJevoisYaxis.toInt();
      //Serial.println(JevoisYaxis);
      JevoisConditionY = true;
    }
    newData = false;
  }
}
