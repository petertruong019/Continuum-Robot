boolean LEDSTATE = false;
boolean ReceivingSerialMessage = false;
boolean NewData = false;
char startMarker = '<';
char endMarker = '>';
const byte numChars = 32;
char ReceivedChars[numChars];
char rc;
static byte index = 0;
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  
  while(Serial.available()>0 && NewData == false)
    {
      F_ReceiveSerialMessage();
    }
    F_PrintReceiveSerialMessage();
  
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
          
          if (ReceivedChars[0] == 'B')
          {
            digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      Serial.println("blink");
            LEDSTATE = true;
          }

          
}
