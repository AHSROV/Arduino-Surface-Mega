#define LCD Serial2
#define MAXPACKETLEN 6

#include <XBOXUSB.h>
// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
XBOXUSB Xbox(&Usb);

//Program Description
//Full of Swag - Mebin

char msgBuffer[255];
int charsInBuffer=0;

bool MotorsKill;

// Pins
int motorPin[] = {
  7, 6, 5, 2, 8, 3};
int reversePin[] = {
  29, 27, 31, 25, 33, 23};
int forwardPin[] = {
  26, 30, 28, 22, 24, 32};

// Motor Information
int motorValue[6] = {
  0, 0, 0, 0, 0, 0};
boolean directionState[8] = {
  false, false, false, false, false, false};
  
float motorPercentage = 1;
int controlSet = 0;
  
String inputString;

void setup()
{
  pinMode(53, INPUT_PULLUP);  
  LCD.begin(9600);
  Serial.begin(9600);
  
  delay(500);
  LCD_Clear_Screen();
  SetPinModes();
    
  KillMotors(); 
  
  TCCR3B = TCCR3B & 0b11111000 | 0x04;
  TCCR4B = TCCR4B & 0b11111000 | 0x04;
  
  delay(200);
  
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
    Serial.print(F("\r\nXBOX USB Library Started"));
}

unsigned long LCDtimer; 

void loop()
{
  if(millis() > LCDtimer) {
    LCDtimer = millis() + 200;
    update_LCD();
  }
  
  if(digitalRead(53))
    KillMotors();
  else
    MotorsKill = false;
  
  Usb.Task();
  if(!Xbox.Xbox360Connected)
  {
    if(ReadSerialStream())
      ProcessMotorCommand(inputString);
  }
  else
  {
    ProcessController();
  }
}

/// Start of packet character: .
/// End of packet character: *
boolean ReadSerialStream(void)
{ 

  while (Serial.available() > 0)
  {
    char c = (char)Serial.read(); // Incoming byte
    
    if (c == '*') // If the char is the end of the packet, stop here and process it
    { 
      return true; // leave so main loop can process it
    }
    
    if (c == '.')  // beginning of packet
    {
      inputString = ""; //clear the current buffer; we're starting a new packet
    }
    else
    {
      inputString += c; // if it's not a start or finish character, then add it to the buffer
    } 
    if (inputString.length() > MAXPACKETLEN) // the packet is longer than any packet we'd expect, throw it out
    {
      inputString = ""; // clear the input string, because obviously this is invalid
    }
  }
  return false;
}

void ProcessMotorCommand(String command)
{ 
  int motor;
  boolean forward;
  int speed; 
  
  // Find the motor number
  motor = command.substring(1, 2).toInt();
  
  // Validate the motor number
  if ((motor < 0) || (motor > 5))
  {
    Serial.println("e: invalid motor.");
    return;
  }
  
  // Find & validate the direction
  if(command[2] == 'f')
    forward = true;
  else if(command[2] == 'r')
    forward = false;
  else
  {
    Serial.println("e: invalid direction");
    return;
  }
  
  // Find the speed
  speed = command.substring(3,6).toInt();
  
  // Validate the speed
  if((speed < 0) || (speed > 100))
  {
    Serial.println("e: invalid speed");
    return;
  }
  
  SetMotor(speed, motor, forward);
}  

int SetMotor(int speed, int motor, boolean forward)
{
  if(speed>=0 || speed<=100)
  {
    motorValue[motor] = (int)(speed*2.55);
  }

  directionState[motor] = forward;

  UpdateMotorSpeed(motor);
  UpdatedirectionState(motor);
}

void SetPinModes()
{
  UpdateMotorSpeeds();
  UpdatedirectionStates();

  for(int i = 0; i < 6; i++)
  {
    pinMode(motorPin[i], OUTPUT);
    pinMode(forwardPin[i], OUTPUT);
    pinMode(reversePin[i], OUTPUT);
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
  if(MotorsKill)
    analogWrite(motorPin[i], 255);
  else
    analogWrite(motorPin[i], 255-motorValue[i]);
/*  Serial.print("Pin ");
  Serial.print(motorPin[i]);
  Serial.print(" set to ");  
  Serial.println(motorValue[i]);*/
}

void UpdatedirectionStates()
{
  for(int i = 0; i < 6; i++)
  {
    UpdatedirectionState(i);
  }
}

void UpdatedirectionState(int i)
{
  digitalWrite(forwardPin[i], directionState[i]);
  digitalWrite(reversePin[i], !directionState[i]);
}

void KillMotors()
{
    MotorsKill = true;
    UpdateMotorSpeeds();
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
  LCD_goto(1,0);
    LCD.print(motorPercentage);
  LCD_goto(1, 19);
  if(MotorsKill)
    LCD.print("X");
  else
    LCD.print(" ");
    
  LCD_goto(1, 5);
  if(controlSet == 0)
    LCD.print("F");
  if(controlSet == 1)
    LCD.print("B");
  
  LCD_goto(2, 0);
  LCD.print("M1:");
  LCD.print(GetSign(directionState[0]));
  LCD.print(motorValue[0]);
  LCD.print("  ");
  
  LCD_goto(2, 10);
  LCD.print("M2:");
  LCD.print(GetSign(directionState[1]));
  LCD.print(motorValue[1]);
  LCD.print("  ");
  
  LCD_goto(3, 0);
  LCD.print("M3:");
  LCD.print(GetSign(directionState[2]));
  LCD.print(motorValue[2]);
  LCD.print("  ");
  
  LCD_goto(3, 10);
  LCD.print("M4:");
  LCD.print(GetSign(directionState[3]));
  LCD.print(motorValue[3]);
  LCD.print("  ");
  
  LCD_goto(4, 0);
  LCD.print("M5:");
  LCD.print(GetSign(directionState[4]));
  LCD.print(motorValue[4]);
  LCD.print("  ");
  
  LCD_goto(4, 10);
  LCD.print("M6:");
  LCD.print(GetSign(directionState[5]));
  LCD.print(motorValue[5]);
  LCD.print("  ");
}

String GetSign(bool direction)
{
  if(direction)
    return " ";
  else
    return "-";
}
