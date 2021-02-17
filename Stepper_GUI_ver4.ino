/* Encoder Notes:
 *  PPR(Resolution - Pulses per Rev) = 200
 *  CPR(Counts per Rev) = PPR*4 = 800
 */

//---------------------------------------------------- LIBRARIES --------------------------------------------------------------------
#include <AccelStepper.h>

//---------------------------------------------------- HARDWARE PIN VARIABLES -------------------------------------------------------
#define stepper1_step_pin 28
#define stepper1_dir_pin 30
#define stepper2_step_pin 6
#define stepper2_dir_pin 7

#define encoderA1 2 //interrupt pin
#define encoderB1 22
#define encoderA2 3 //interrupt pin
#define encoderB2 24

// --------- DEFINE STEPPER MOTORS IN THE ACCELSTEPPER LIBRARY - (BIPOLAR,STEP_PIN,DIR_PIN)------------------------------------------
AccelStepper stepper1 (1, stepper1_step_pin, stepper1_dir_pin); 
AccelStepper stepper2 (1, stepper2_step_pin, stepper2_dir_pin); 

// ------------------------------------------------- GLOBAL VARIABLES ---------------------------------------------------------------
String data[6]; 
int index = 0;
int enc_pos1 = 0;
int enc_pos2 = 0;

// ----------------------------------------------- STEPPER-RELATED VARIABLES --------------------------------------------------------
float StepsPerRev = 200.0;//change according to stepper motor being used. Check stepper motor datasheet
float theta1 = 0.0;
float theta2 = 0.0;
float stepper_speed1 = 0;
float stepper_speed2 = 0;

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
  pinMode(encoderA2, INPUT);
  pinMode(encoderB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA1),encoder1,FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderA2),encoder2,FALLING);
  
}

// ------------------------------------------------- MAIN LOOP ---------------------------------------------------------------------
void loop() 
{
  int enc_pos_read1 = 0;
  int enc_pos_read2 = 0;
 
  
  while(Serial.available()>0) //opens the serial port
  {
    delay(10);
    
    if(Serial.available()>0)//Checks to see if anything is in the serial port
    {
      
      data[index] = Serial.readStringUntil(',');
      index++; 
      
      if (index >= 5)
      {

        // STRING ARRAY SERIAL PORT CONVENTION:
        // [DIRECTION, THETA POSITION 1, STEPPER 1 SPEED, THETA POSITION 2, STEPPER 2 SPEED,... etc.] 
        index = 0; //resets the index back to zero        
        theta1 = (StepsPerRev/360)*data[1].toInt(); //.toInt() converts data[1] type into an integer type
        stepper_speed1 = data[2].toInt();
        theta2 = (StepsPerRev/360)*data[3].toInt();
        stepper_speed2 = data[4].toInt();

        if(data[0] == "C")
        {

          stepper1.setMaxSpeed(stepper_speed1);
          stepper1.moveTo(theta1);
          stepper2.setMaxSpeed(stepper_speed2);
          stepper2.moveTo(theta2);
          
          while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0)
          {
            if (enc_pos1 != enc_pos_read1 || enc_pos2 != enc_pos_read2)
            {
              enc_pos_read1 = enc_pos1;
              enc_pos_read2 = enc_pos2;
              Serial.print(enc_pos_read1); Serial.print(",");
              Serial.print(enc_pos_read2); Serial.println(",");
              
              
            }
            stepper1.run();
            stepper2.run();
              
          }
            stepper1.setCurrentPosition(0); //resets the position of the stepper motor
            stepper2.setCurrentPosition(0);
        }              
       }
      
      }  
  }
}


// ----------------------------------------------- FUNCTIONS -------------------------------------------------------
void encoder1()
{
  if (digitalRead(encoderB1) == LOW) 
  {
    enc_pos1++;
  }
  else
  {
    enc_pos1--;
  }
}

void encoder2()
{
  if (digitalRead(encoderB2) == LOW) 
  {
    enc_pos2++;
  }
  else
  {
    enc_pos2--;
  }
}
