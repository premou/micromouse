#include <EEPROM.h>
#include <Encoder.h>

// Encoder pulses to mm
Encoder myEnc(0, 1);
long divider = 1632;
long oldPosition  = 0;
long offsetPosition  = 0;

// Standard PWM DC control
int E1 = 5;     //M1 Speed Control
int M1 = 4;    //M1 Direction Control

void stop(void)                    //Stop
{
  digitalWrite(E1, LOW);
}
void advance(char a)          //Move forward
{
  analogWrite (E1, a);     //PWM Speed Control
  digitalWrite(M1, HIGH);
}
void back_off (char a)          //Move backward
{
  analogWrite (E1, a);
  digitalWrite(M1, LOW);
}

// Keys
int  adc_key_val[5] ={
  30, 150, 360, 535, 760 };
int NUM_KEYS = 5;
int adc_key_in;
int key=-1;
int oldkey=-1;

// Convert ADC value to key number
int get_key(unsigned int input)
{   
  int k;
  for (k = 0; k < NUM_KEYS; k++)
  {
    if (input < adc_key_val[k])
    {  
      return k;  
    }
  }
  if (k >= NUM_KEYS)
    k = -1;     // No valid key pressed
  return k;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(-newPosition/divider);
  }

  if (Serial.available()) {
    char val = Serial.read();
    if (val != -1)
    {
      switch (val)
      {
        case '0': // ZERO : reset position to zero
          myEnc.write(0);
          break;
        case 'O' : // ORIGIN : go to position zero
          while (myEnc.read() > 100 || myEnc.read() < -100)
          {
            if (myEnc.read() > 1000)
              back_off (255);
            if (myEnc.read() > 100)
              back_off (255);
            if (myEnc.read() < -1000)
              advance (255);
            if (myEnc.read() < -100)
              advance (255);
          }
          stop();
          break;
        case 'B': // Move Backward
          back_off (255);   //move back in max speed
          break;
        case 'b': // Move Backward
          back_off (100);   //move back in max speed
          break;
        case 'F': // Move Forward
          advance (255);   //move forward in max speed
          break;
        case 'f': // Move Forward
          advance (100);   //move forward in max speed
          break;
        case '!': // PING-PONG
          Serial.println("!");
          break;
        case 'S':
        //default:
          stop();
          break;
      }
    }
    else stop();
  }
  
  adc_key_in = analogRead(0);    // read the value from the sensor  
  /* get the key */
  key = get_key(adc_key_in);    // convert into key press
  if (key != oldkey) {   // if keypress is detected
    delay(50);      // wait for debounce time
    adc_key_in = analogRead(0);    // read the value from the sensor  
    key = get_key(adc_key_in);    // convert into key press
    if (key != oldkey) {         
      oldkey = key;
      if(key==-1)
        stop();
      else if (key >=0){
        //Serial.println(adc_key_in, DEC);
        switch(key)
        {
          case 0:
          myEnc.write(0);
          break;
          case 1:
          back_off (255);          
          break;
          case 2:
          back_off (100);
          break;
          case 3:
          advance (100);
          break;
          case 4:
          advance (255);
          break;
          }
      }
    }
  }  
}
