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
  
  if(Serial)
  {
    if(digitalRead(53))
      KillMotors();
    else
      MotorsKill = false;
    
    if(Serial)
    {
      if(ReadSerialStream())
        ProcessMotorCommand(inputString);
    }
    else
    {
      ProcessController();
    }
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

void ProcessController()
{
  float motorUpBack, motorUpFront, motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
  
  Usb.Task();
  if(Xbox.Xbox360Connected)
  {
    // Up Down controlled by triggers
    motorUpBack = NormalLongToFloat(Xbox.getButtonPress(L2) * 128) - NormalLongToFloat(Xbox.getButtonPress(R2)*128);
    motorUpFront = NormalLongToFloat(Xbox.getButtonPress(L2) * 128) - NormalLongToFloat(Xbox.getButtonPress(R2)*128);
    
    motorFrontLeft = -NormalLongToFloat(Xbox.getAnalogHat(LeftHatX));
    motorFrontRight = NormalLongToFloat(Xbox.getAnalogHat(LeftHatX));
    motorBackLeft = -NormalLongToFloat(Xbox.getAnalogHat(LeftHatX));
    motorBackRight = NormalLongToFloat(Xbox.getAnalogHat(LeftHatX));
    
    //forward and backward movement it controlled by the left y-axis
    motorFrontRight += -NormalLongToFloat(Xbox.getAnalogHat(LeftHatY));
    motorFrontLeft  += -NormalLongToFloat(Xbox.getAnalogHat(LeftHatY));
    motorBackLeft   += NormalLongToFloat(Xbox.getAnalogHat(LeftHatY));
    motorBackRight  += NormalLongToFloat(Xbox.getAnalogHat(LeftHatY));

    //rotation is controlled by the right thumbstick x-axis
    motorFrontLeft  += -NormalLongToFloat(Xbox.getAnalogHat(RightHatX));
    motorFrontRight += NormalLongToFloat(Xbox.getAnalogHat(RightHatX));
    motorBackLeft   += NormalLongToFloat(Xbox.getAnalogHat(RightHatX));
    motorBackRight  += -NormalLongToFloat(Xbox.getAnalogHat(RightHatX));

    //Pitch is controlled by the right thumbstick y-axis
    motorUpBack  += NormalLongToFloat(Xbox.getAnalogHat(RightHatY));
    motorUpFront += -NormalLongToFloat(Xbox.getAnalogHat(RightHatY));
    
    //find the largest value for the lateral motors
    float maxLateral = max(1, abs(motorFrontLeft));
    maxLateral = max(maxLateral, abs(motorFrontRight));
    maxLateral = max(maxLateral, abs(motorBackRight));
    maxLateral = max(maxLateral, abs(motorBackLeft));

    //divide by the largest number
    motorBackLeft   /= maxLateral;
    motorBackRight  /= maxLateral;
    motorFrontLeft  /= maxLateral;
    motorFrontRight /= maxLateral;

    //Find the largest value for the up/downs
    float maxUp = max(1, abs(motorUpBack));
    maxUp = max(maxUp, abs(motorUpFront));

    //Divide by the largest number
    motorUpBack  /= maxUp;
    motorUpFront /= maxUp;
  }
  int motorInt[6] = {};
  
  motorInt[5] = (motorUpFront * 100);
  motorInt[4] = (motorUpBack * 100);
  motorInt[0] = (motorFrontLeft * 100);
  motorInt[1] = (motorFrontRight * 100);
  motorInt[2] = (motorBackLeft * 100);
  motorInt[3] = (motorBackRight * 100);
  
  for(int i = 0; i < 6; i++)
  {
    bool negative;
    if(motorInt[i] < 0)
      negative = true;
    else
      negative = false;
    /*Serial.print("Motor: ");
    Serial.print(i);
    Serial.print(" now running at ");
    Serial.println(motorInt[i]);*/
    SetMotor(motorInt[i], i, negative);
  }
}

float NormalLongToFloat(long input) 
{
  float temp; 
  temp = (float) input / 32700.0;
  if(temp > 1.0)
    temp = 1.0;
  if(temp < -1.0)
    temp = -1.0;
    
  if(temp < .05 && temp > -.05)
    temp = 0;
    
  return temp;
}

int SetMotor(int speed, int motor, boolean forward)
{
  if(speed>=0 || speed<=100)
  {
    motorValue[motor] = (int)(speed*2.55);
  }

  directionState[motor] = forward;

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
  LCD_goto(1, 0);
  if(MotorsKill)
    LCD.print("Motors Killedo!        ");
  else
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

