#define LCD Serial2
#define MAXPACKETLEN 6

//Program Description
//Full of Swag - Mebin

char msgBuffer[255];
int charsInBuffer=0;

bool MotorsKill;

// Pins
int motorPin[] = {
  2, 3, 5, 6, 7, 8};
int forwardPin[] = {
  22, 24, 26, 28, 30, 32};
int reversePin[] = {
  23, 25, 27, 29, 31, 33};

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
  
  if (Serial.available() > 0)
  {  
    ReadSerialStream();
    ProcessMotorCommand(inputString);
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
      return false;
    }
    else
    {
      inputString += c; // if it's not a start or finish character, then add it to the buffer
      return false;
    } 
    if (inputString.length() > MAXPACKETLEN) // the packet is longer than any packet we'd expect, throw it out
    {
      inputString = ""; // clear the input string, because obviously this is invalid
      return false;
    }
  }
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
  {
    if(i > 3)
      analogWrite(motorPin[i], 0);
    else
      analogWrite(motorPin[i], 255);
  }
  else
  {
    if(i > 3)
      analogWrite(motorPin[i], motorValue[i]);
    else
      analogWrite(motorPin[i], 255-motorValue[i]);
  }
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

