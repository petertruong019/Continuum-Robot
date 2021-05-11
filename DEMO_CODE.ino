// ---------------------- Revisions --------------------------------------
// Revision 1: Preliminary Version. Code is taken from DCMotorControlPID.ino ver 4. Uses Jevois camera feedback instead of encoders.
// Revision 2: Added joystick manual control loop.
// Revision 3: Added circle pattern PID.
// Revision 4: Implement IMU homing function.
// Revision 5: Added serial protocol for Pi integration.
// ----------------------------------------------------------------------
#include <Wire.h>
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
#define JOYSTICK_SW A2

int pos1 = 0; // This would be the y axis
int pos2 = 0; // This would be the x axis
int target1 = 0;
int target2 = 0;
int dir1 = 0;
int dir2 = 0;
long prevTime1 = 0;
long prevTime2 = 0;
long prevTime3 = 0;
float e1 = 0;
float e2 = 0;
float eprev1 = 0;
float eprev2 = 0;
float eprev3 = 0;
float eintegral1 = 0;
float eintegral2 = 0;
float etinegral3 = 0;
float pi = 3.14;
int JoyPosition_X = 0;
int JoyPosition_Y = 0;

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
int i;

// ---------------------------------------------- IMU VARIABLES ----------------------------------------------- //
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float elapsedTime, time, timePrev;
float rad_to_deg = 180 / 3.141592654;
float desired_angle = 0; //This is the angle in which we whant the
//balance to stay steady

// ----------------------------------------------- SERIAL COMMUNICATION-RELATED VARIABLES --------------------------------------------------------
float DCMotorPosition1 = 0.0;
float DCMotorPosition2 = 0.0;

float JevoisXaxis = 0.0;
float JevoisYaxis = 0.0;

String StrDCMotorPosition1;
String StrDCMotorPosition2;

boolean JevoisMode = false;
boolean JevoisConditionX = false;
boolean JevoisConditionY = false;

String StrJevoisXaxis;
String StrJevoisYaxis;

boolean MotorPositionCondition1 = false;
boolean MotorPositionCondition2 = false;

boolean ManualEntryMode = false;
boolean JoyStickMode = false;

boolean PatternMode = false;
boolean CirclePattern = false;
boolean CircleMove = false;
boolean CircleHome = false;
boolean SquarePattern = false;
boolean SquareMove = false;
boolean RibbonPattern = false;
boolean RibbonMove = false;

boolean HomingMode = false;

// -----------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  TCCR4B = TCCR4B & B11111000 | B00000001; // This sets the PWM frequency to 32khz
  TCCR2B = TCCR2B & B11111000 | B00000001;
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
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

  time = millis(); //Start counting time in milliseconds
  delay(1000);
  //Serial.println("target pos");
}

// ------------------------------------------------------------- MAIN LOOP -------------------------------------------------
void loop() {
  ReceiveSerialArduino();
  ReceiveSerialJevois();
  F_CheckSerialProtocol();
  F_RunJevoisArduino();
  F_RunDCPID();
  F_RunCirclePID();
  F_RunSquarePID();
  F_RunRibbonPID();
  F_RunJoyStick();
  F_Homing();
  newData = false;
}

// ----------------------------------------------- JEVOIS CAMERA FUNCTIONS FOR ARDUINO INTERFACE -----------------------------------------------------------------
void F_RunJevoisArduino()
{
  if (JevoisMode == true && JevoisConditionX == true && JevoisConditionY == true)
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
    //    Serial.print(target1); Serial.print(" "); Serial.print(target2); Serial.print(" "); Serial.print(JevoisYaxis); Serial.print(" "); Serial.print(JevoisXaxis); Serial.println();
    //Serial.print("Time Elapsed: "); Serial.println(deltaT1);
  }
}

//------------------------------------------------ IMU HOMING FUNCTION ----------------------------------------------------
void F_Homing()
{
  if (HomingMode == true)
  {
    {
      timePrev = time;  // the previous time is stored before the actual time read
      time = millis();  // actual time read
      elapsedTime = (time - timePrev) / 1000;

      Wire.beginTransmission(0x68);
      Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 6, true);

      Acc_rawX = Wire.read() << 8 | Wire.read(); //each value needs two registres
      Acc_rawY = Wire.read() << 8 | Wire.read();
      Acc_rawZ = Wire.read() << 8 | Wire.read();
      /*---X---*/
      Acceleration_angle[0] = atan((Acc_rawY / 16384.0) / sqrt(pow((Acc_rawX / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * rad_to_deg;
      /*---Y---*/
      Acceleration_angle[1] = atan(-1 * (Acc_rawX / 16384.0) / sqrt(pow((Acc_rawY / 16384.0), 2) + pow((Acc_rawZ / 16384.0), 2))) * rad_to_deg;

      Wire.beginTransmission(0x68);
      Wire.write(0x43); //Gyro data first adress
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 4, true); //Just 4 registers

      Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
      Gyr_rawY = Wire.read() << 8 | Wire.read();

      /*---X---*/
      Gyro_angle[0] = Gyr_rawX / 131.0;
      /*---Y---*/
      Gyro_angle[1] = Gyr_rawY / 131.0;

      /*---X axis angle---*/
      Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0] * elapsedTime) + 0.02 * Acceleration_angle[0];
      float pitch_error = Total_angle[0];
      /*---Y axis angle---*/
      Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_angle[1] * elapsedTime) + 0.02 * Acceleration_angle[1];
      float yaw_error = Total_angle[1];

      Serial.print(pitch_error); Serial.print(" "); Serial.println(yaw_error);

      int target1 = 0;
      int target2 = 0;

      //PID GAINS FOR MOTOR 1
      float kp1 = 70;
      float kd1 = 0.65;
      float ki1 = 0;

      //PID GAINS FOR MOTOR 2
      float kp2 = 70;
      float kd2 = 0.65;
      float ki2 = 0;

      // TIME DIFFERENCE -----------------------------------------------
      long currTime1 = micros();
      long currTime2 = micros();
      float deltaT1 = ((float) (currTime1 - prevTime1)) / ( 1.0e6 );
      float deltaT2 = ((float) (currTime2 - prevTime2)) / ( 1.0e6 );
      prevTime1 = currTime1;
      prevTime2 = currTime2;
      // ERROR ---------------------------------------------------------
      float e1 = yaw_error - target1;
      float e2 = pitch_error - target2;
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
      if (pwr1 < 125) {
        pwr1 = 125;
      }
      if (pwr2 < 125) {
        pwr2 = 125;
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
      if (fabs(e1) <= 0.5 && fabs(e2) <= 0.5)
      {
        HomingMode = false;
        Serial.print("Homing done!");
        //      Serial.print(pos1); Serial.print(" "); Serial.println(pos2);
        pos1 = 0;
        pos2 = 0;
        pwr1 = 0;
        pwr2 = 0;
        //      Serial.print(pos1); Serial.print(" "); Serial.println(pos2);
      }
      // ASSIGN SIGNAL TO MOTORS ---------------------------------------
      setMotor1(dir1, pwr1, PWM1, IN1, IN2);
      setMotor2(dir2, pwr2, PWM2, IN3, IN4);
      // STORE PREVIOUS ERROR ------------------------------------------
      eprev1 = e1;
      eprev2 = e2;
    }
  }
}

//------------------------------------------------ CIRCLE PATTERN ----------------------------------------------------------
void F_RunCirclePID()
{
  if (PatternMode == true && CirclePattern == true)
  {
    //PID GAINS FOR MOTOR 1
    float kp1 = 175;
    float kd1 = .0674;
    float ki1 = 0;

    //PID GAINS FOR MOTOR 2
    float kp2 = 175;
    float kd2 = .0674;
    float ki2 = 0;
    int cy;
    int cr = 700;

    for (float cdeg = 0; cdeg <= 360; cdeg = cdeg + 0.5)
    {
      float crad = cdeg * (pi / 180);
      double cy = sin(crad);
      double cx = cos(crad);
      int target1 = cy * cr;
      int target2 = cx * cr;
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
        if ( fabs(e1) <= 1 && fabs(e2) <= 1)
        {
          CircleMove = false;
        }
      }
      if (cdeg == 360)
      {
        CircleHome = true;
        while (CircleHome == true)
        {
          target1 = 0;
          target2 = 0;
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
          if ( fabs(e1) <= 1 && fabs(e2) <= 1)
          {
            CircleHome = false;
            Serial.println("Pattern done!");
          }
        }
        int pwr1 = 0;
        int pwr2 = 0;
        setMotor1(dir1, pwr1, PWM1, IN1, IN2);
        setMotor2(dir2, pwr2, PWM2, IN3, IN4);
        CirclePattern = false;
      }
    }
  }
}

// -------------------------------------------------------- SQUARE PATTERN ------------------------------------------------------ //
void F_RunSquarePID()
{
  if (PatternMode == true && SquarePattern == true)
  {
    //PID GAINS FOR MOTOR 1
    //float kp1 = 167.5;
    //float kd1 = 1.25;
    //float ki1 = 0;

    //PID GAINS FOR MOTOR 2
    //float kp2 = 167.5;
    //float kd2 = 1.25;
    //float ki2 = 0;

    //PID GAINS FOR MOTOR 1
    float kp1 = 175;
    float kd1 = .0674;
    float ki1 = 0;

    //PID GAINS FOR MOTOR 2
    float kp2 = 175;
    float kd2 = .0674;
    float ki2 = 0;

    int sy = 700;
    int sx = sy;
    int syCords[7] = {0, sy, sy, -sy, -sy, 0, 0};
    int sxCords[7] = {sx, sx, -sx, -sx, sx, sx, 0};

    for (int i = 0; i <= 6; i++)
    {
      int target1 = syCords[i];
      int target2 = sxCords[i];
      SquareMove = true;
      while (SquareMove == true)
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
        if ( fabs(e1) <= 1 && fabs(e2) <= 1)
        {
          SquareMove = false;
        }
      }
      if (i == 6)
      {
        int pwr1 = 0;
        int pwr2 = 0;
        setMotor1(dir1, pwr1, PWM1, IN1, IN2);
        setMotor2(dir2, pwr2, PWM2, IN3, IN4);
        SquarePattern = false;
        Serial.println("Pattern done!");
      }
    }
  }
}

// -------------------------------------------------------- RIBBON PATTERN ------------------------------------------------------ //
void F_RunRibbonPID()
{
  if (PatternMode == true && RibbonPattern == true)
  {
    //PID GAINS FOR MOTOR 1
    //float kp1 = 167.5;
    //float kd1 = 1.25;
    //float ki1 = 0;

    //PID GAINS FOR MOTOR 2
    //float kp2 = 167.5;
    //float kd2 = 1.25;
    //float ki2 = 0;
    //PID GAINS FOR MOTOR 1
    //PID GAINS FOR MOTOR 1
    float kp1 = 175;
    float kd1 = .0674;
    float ki1 = 0;

    //PID GAINS FOR MOTOR 2
    float kp2 = 175;
    float kd2 = .0674;
    float ki2 = 0;
    int hy = 700;
    int hx = hy;
    int hyCords[5] = {hy, hy, -hy, -hy, 0};
    int hxCords[5] = {hx, -hx, hx, -hx, 0};

    for (int i = 0; i <= 4; i++)
    {
      int target1 = hyCords[i];
      int target2 = hxCords[i];
      RibbonMove = true;
      while (RibbonMove == true)
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
        if ( fabs(e1) <= 1 && fabs(e2) <= 1)
        {
          RibbonMove = false;
        }
      }
      if (i == 4)
      {
        int pwr1 = 0;
        int pwr2 = 0;
        setMotor1(dir1, pwr1, PWM1, IN1, IN2);
        setMotor2(dir2, pwr2, PWM2, IN3, IN4);
        RibbonPattern = false;
        Serial.println("Pattern done!");
      }
    }
  }
}

// ----------------------------------------------------------------------- JOYSTICK MANUAL CONTROL ------------------------------------------------
void F_RunJoyStick()
{
  if (JoyStickMode == true)
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
    //    Serial.print(pos1); Serial.print(" "); Serial.println(pos2);
    //  Serial.print("X: ");
    //  Serial.print(JoyPosition_X); Serial.print(" "); Serial.print(pwr1); Serial.print(" "); Serial.print(dir1); Serial.print(" ");
    //  Serial.print("Y: ");
    //  Serial.print(JoyPosition_Y); Serial.print(" "); Serial.print(pwr2); Serial.print(" "); Serial.println(dir2);
  }
}

// --------------------------------------------------------------------- MANUAL POSITION ENTRY --------------------------------------------------
void F_RunDCPID()
{
  if (ManualEntryMode == true && MotorPositionCondition1 == true && MotorPositionCondition2 == true)
  {
    // set target position
    int target1 = DCMotorPosition1;
    int target2 = DCMotorPosition2;

    //PID GAINS FOR MOTOR 1
    float kp1 = 175;
    float kd1 = .0674;
    float ki1 = 0;

    //PID GAINS FOR MOTOR 2
    float kp2 = 175;
    float kd2 = .0674;
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

    //    Serial.print(target1); Serial.print(" "); Serial.print(target2); Serial.print(" "); Serial.print(pos1); Serial.print(" "); Serial.print(pos2); Serial.println(" ");

  }
}

// ------------------------------------- DC MOTOR AND ENCODER CONTROL FUNCTIONS --------------------------------------------------
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
  int a1 = digitalRead(ENCA1);
  int b1 = digitalRead(ENCB1);
  if (a1 != b1)
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
  int a2 = digitalRead(ENCA2);
  int b2 = digitalRead(ENCB2);
  if (a2 != b2)
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

//void ShowJevoisXY()
//{
//  if (newData == true)
//  {
//    Serial.println(receivedChars);
//  }
//}

void F_CheckSerialProtocol()
{
  if (newData == true)
  {
    // ------------------------------- TRIGGERS FOR DC MOTOR POSITION ENTRY --------------------- //
    if (receivedChars[0] == 'M' && receivedChars[1] == 'M' && receivedChars[2] == '1')
    {
      ManualEntryMode = true;//Manual Entry On
      int pos1 = 0;
      int pos2 = 0;
      //JoyStickMode = false;
      //      Serial.println("MM1");
    }
    else if (receivedChars[0] == 'M' && receivedChars[1] == 'M' && receivedChars[2] == '0')
    {
      ManualEntryMode = false;//Manual Entry OFF
      int dir1 = 0;
      int dir2 = 0;
      int pwr1 = 0;
      int pwr2 = 0;
      int pos1 = 0;
      int pos2 = 0;
      setMotor1(dir1, pwr1, PWM1, IN1, IN2);
      setMotor2(dir2, pwr2, PWM2, IN3, IN4);
      //      Serial.println("MM0");
    }
    else if (receivedChars[0] == '1' && receivedChars[1] == 'p')
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
    // --------------------------------- TRIGGERS FOR OBJECT TRACING MODE AND JEVOIS DATA RECEIVE/SEND ------------------------- //
    else if (receivedChars[0] == 'O' && receivedChars[1] == 'T' && receivedChars[2] == '1')
    {
      JevoisMode = true;//JEVOIS On
      //Serial.println("OT1");
    }
    else if (receivedChars[0] == 'O' && receivedChars[1] == 'T' && receivedChars[2] == '0')
    {
      JevoisMode = false;//JEVOIS OFF
      int dir1 = 0;
      int dir2 = 0;
      int pwr1 = 0;
      int pwr2 = 0;
      int pos1 = 0;
      int pos2 = 0;
      setMotor1(dir1, pwr1, PWM1, IN1, IN2);
      setMotor2(dir2, pwr2, PWM2, IN3, IN4);
      //Serial.println("OT0");
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
    else if (JevoisMode == true && receivedChars[0] == 'T' && receivedChars[1] == 'A' && receivedChars[2] == 'R')
    {
      Serial1.print("target"); Serial1.print("\n");
    }
    else if (JevoisMode == true && receivedChars[0] == 'C' && receivedChars[1] == 'A' && receivedChars[2] == 'L')
    {
      Serial1.print("calibration"); Serial1.print("\n");
      int dir1 = 0;
      int dir2 = 0;
      int pwr1 = 0;
      int pwr2 = 0;
      int pos1 = 0;
      int pos2 = 0;
      setMotor1(dir1, pwr1, PWM1, IN1, IN2);
      setMotor2(dir2, pwr2, PWM2, IN3, IN4);
    }
    else if (JevoisMode == true && receivedChars[0] == 'O' && receivedChars[1] == 'B' && receivedChars[2] == 'S')
    {
      Serial1.print("obstacle"); Serial1.print("\n");
      int dir1 = 0;
      int dir2 = 0;
      int pwr1 = 0;
      int pwr2 = 0;
      int pos1 = 0;
      int pos2 = 0;
      setMotor1(dir1, pwr1, PWM1, IN1, IN2);
      setMotor2(dir2, pwr2, PWM2, IN3, IN4);
    }
    else if (JevoisMode == true && receivedChars[0] == 'G' && receivedChars[1] == 'R')
    {
      Serial1.print("green"); Serial1.print("\n");
    }
    else if (JevoisMode == true && receivedChars[0] == 'R' && receivedChars[1] == 'E' && receivedChars[2] == 'D')
    {
      Serial1.print("red"); Serial1.print("\n");
    }
    else if (JevoisMode == true && receivedChars[0] == 'B' && receivedChars[1] == 'L')
    {
      Serial1.print("blue"); Serial1.print("\n");
    }

    // ----------------------------- TRIGGERS FOR HOMING MODE ----------------------------- //
    else if (receivedChars[0] == 'H')
    {
      HomingMode = true; //Run homing function
    }
    // --------------------------------------- TRIGGERS FOR JOYSTICK MODE ---------------------------- //
    else if (receivedChars[0] == 'J' && receivedChars[1] == 'S' && receivedChars[2] == '1')
    {
      JoyStickMode = true; //Joystick On
    }
    else if (receivedChars[0] == 'J' && receivedChars[1] == 'S' && receivedChars[2] == '0')
    {
      JoyStickMode = false; //Joystick On
    }
    // -------------------------------------- TRIGGERS FOR PATTERN MODE ------------------------------ //
    else if (receivedChars[0] == 'P' && receivedChars[1] == 'M' && receivedChars[2] == '1')
    {
      PatternMode = true;//Pattern On
    }
    else if (receivedChars[0] == 'P' && receivedChars[1] == 'M' && receivedChars[2] == '0')
    {
      PatternMode = false;//Pattern OFF
      int dir1 = 0;
      int dir2 = 0;
      int pwr1 = 0;
      int pwr2 = 0;
      int pos1 = 0;
      int pos2 = 0;
      setMotor1(dir1, pwr1, PWM1, IN1, IN2);
      setMotor2(dir2, pwr2, PWM2, IN3, IN4);
      //Serial.println("PM0");
    }
    else if (receivedChars[0] == 'C' && receivedChars[1] == 'P' && receivedChars[2] == '1')
    {
      CirclePattern = true;
    }
    else if (receivedChars[0] == 'C' && receivedChars[1] == 'P' && receivedChars[2] == '0')
    {
      CirclePattern = false;
    }
    else if (receivedChars[0] == 'S' && receivedChars[1] == 'P' && receivedChars[2] == '1')
    {
      SquarePattern = true;
    }
    else if (receivedChars[0] == 'S' && receivedChars[1] == 'P' && receivedChars[2] == '0')
    {
      SquarePattern = false;
    }
    else if (receivedChars[0] == 'R' && receivedChars[1] == 'P' && receivedChars[2] == '1')
    {
      RibbonPattern = true;
    }
    else if (receivedChars[0] == 'R' && receivedChars[1] == 'P' && receivedChars[2] == '0')
    {
      RibbonPattern = false;
    }
    newData = false;
  }
}
