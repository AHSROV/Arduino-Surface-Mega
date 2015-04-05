#include <SPI.h>

#define LCD Serial2
#define MAXPACKETLEN 6

//Program Description
//Full of Swag - Mebin

char msgBuffer[255];
int charsInBuffer=0;

// Pins
int motorPin[] = {
  2, 3, 4, 5, 6, 7};
int forwardPin[] = {
  22, 24, 26, 28, 30, 32};
int reversePin[] = {
  23, 25, 27, 29, 31, 33};

// Motor Information
int motorValue[6] = {
  0, 0, 0, 0, 0, 0};
boolean directionState[8] = {
  false, false, false, false, false, false};

void setup()
{

  LCD.begin(9600);
  Serial.begin(9600);
  
  delay(500);
  LCD_Clear_Screen();
  SetPinModes();
}

unsigned long LCDtimer; 

void loop()
{
  if(millis() > 200) {
    LCDtimer = millis();
    update_LCD();
  }

  if (Serial.available() > 0)
  {  
    //these Major Leters will be used for controlling the motor.
    //Read the Data Taken and then converts it into Byte from ASCII Code
    if (charsInBuffer >= sizeof(msgBuffer))
    {
      Serial.println("e: overflow");
      charsInBuffer = 0; 
    } 
    msgBuffer[charsInBuffer] = (byte)Serial.read();
    charsInBuffer++;  

    // Wait for end of line
    if (msgBuffer[charsInBuffer-1] == 10 || msgBuffer[charsInBuffer-1] == 13)
    {
      if(msgBuffer[0] == 'm' && msgBuffer[1] <= '7')
      {
        ProcessMotorCommand(msgBuffer, charsInBuffer);
      }
      else if(msgBuffer[0] == 'p' && msgBuffer[1] == 'u')
      {
        Serial.println("pu");
      }
      else
      {
        msgBuffer[charsInBuffer - 1] = 0;
      }
      charsInBuffer = 0; 
    } 
  } 
}

/// Start of packet character: .
/// End of packet character: *
String Read_Serial_Stream(void)
{
  String inputString; 
  
  while (Serial.available())
  {
    char c = (char)Serial.read(); // Incoming byte
    
    if (c == '*') // If the char is the end of the packet, stop here and process it
    { 
      return inputString; // leave so main loop can process it
    }
    
    if (c == '.')  // beginning of packet
    {
      inputString = ""; //clear the current buffer; we're starting a new packet
    }
    else
      inputString += c; // if it's not a start or finish character, then add it to the buffer
      
    if (inputString.length() > MAXPACKETLEN) // The packet is longer than any packet we'd expect, throw it out
    {
      inputString = ""; // clear the input string, because obviously this is invalid
    }
  }
}

int GetValFromString(char *p, int length)
{
  if (!isDigit(*p) || length < 1)
  {
    return -1;
  }

  int v=0;
  while(isDigit(*p) && length > 0)
  {
    v = v*10 + *p - '0';
    p++; 
    length--;
  }
  return v;
}

void ProcessMotorCommand(char msgBuffer[], int msgLen)
{ 
  if (msgLen < 4)
  {
    Serial.println("e: cmd too short.");
    return;
  }
  int motor = GetValFromString(&msgBuffer[1], 1);
  if ((motor < 0) || (motor > 5))
  {
    Serial.println("e: invalid motor.");
    return;
  }

  //if it starts with M and ends with a % and a LF or CR ... DO what it says below
  //Motor Direction Decleration
  if(msgBuffer[2]=='f')
  {
    // We assume everything but the first letter and the line ending are part of a number.
    int speed = GetValFromString(&msgBuffer[3], charsInBuffer-3);
    if (speed < 0)
    {
      Serial.println("e: error, not an integer.");
      return;
    }
    SendMessage(speed,motor,true);  
  }   
  else if(msgBuffer[2]=='r')
  {
    int speed = GetValFromString(&msgBuffer[3], charsInBuffer-3);
    SendMessage(speed,motor,false);  
  }
  else
  {
    Serial.print("e: Unrecognized cmd: ");
    msgBuffer[charsInBuffer-1] = 0;
    Serial.println(msgBuffer);
  }

  Serial.println();
}


int SendMessage(int speed, int motor, boolean forward)
{
  if(speed>=0 || speed<=100)
  {
    motorValue[motor] = (int)(speed*2.55);
    //Serial.print("Speed: ");
    //Serial.println(motorValue[motor]);
  }

  directionState[motor] = forward;
  //Serial.print("Direction: ");
  //Serial.println(directionState[motor]);

  UpdateMotorSpeed(motor);
  UpdateMotorDirection(motor);
}

void SetPinModes()
{
  UpdateMotorSpeeds();
  UpdateMotorDirections();

  for(int i = 0; i < 6; i++)
  {
    pinMode(motorPin[i], OUTPUT);
    pinMode(forwardPin[i], OUTPUT);
  }
}

void UpdateMotorSpeeds() 
{
  for(int i = 0; i < 6; i++) 
  {
    UpdateMotorSpeed(i);
  }
}

void UpdateMotorSpeed(int i) 
{
  analogWrite(motorPin[i], motorValue[i]);
  Serial.print("Pin ");
  Serial.print(motorPin[i]);
  Serial.print(" set to ");  
  Serial.println(motorValue[i]);
}

void UpdateMotorDirections()
{
  for(int i = 0; i < 6; i++)
  {
    UpdateMotorDirection(i);
  }
}

void UpdateMotorDirection(int i)
{
  digitalWrite(forwardPin[i], directionState[i]);
  digitalWrite(reversePin[i], !directionState[i]);
}


// Pete's Code
void LCD_Clear_Screen(void)
{
  LCD.write( 254 );
  LCD.write( 1 );
  delay(20);
  LCD.write( 254 );
  LCD.write( 12 );

}

/* ------------------------------------------------- */
// LineNum = [1,2]
// cPos = [0..15]

void LCD_goto(uint8_t lineNum, uint8_t cPos)
{
	char c = 0;	
	if (lineNum == 2) c = 64;
	if (lineNum == 3) c = 20;
	if (lineNum == 4) c = 84;

	c += (cPos | 0x80);
	
	LCD.write(254);
	LCD.write(c);
}
// End Pete's Code

void update_LCD() {
  LCD_goto(1, 0);
  LCD.print("AHS ROV Confidential");
  
  LCD_goto(2, 0);
  LCD.print("M1: ");
  LCD.print(motorValue[0]);
  LCD.print("  ");
  
  LCD_goto(2, 10);
  LCD.print("M2: ");
  LCD.print(motorValue[1]);
  LCD.print("  ");
  
  LCD_goto(3, 0);
  LCD.print("M3: ");
  LCD.print(motorValue[2]);
  LCD.print("  ");
  
  LCD_goto(3, 10);
  LCD.print("M4: ");
  LCD.print(motorValue[3]);
  LCD.print("  ");
  
  LCD_goto(4, 0);
  LCD.print("M5: ");
  LCD.print(motorValue[4]);
  LCD.print("  ");
  
  LCD_goto(4, 10);
  LCD.print("M6: ");
  LCD.print(motorValue[5]);
  LCD.print("  ");
}

