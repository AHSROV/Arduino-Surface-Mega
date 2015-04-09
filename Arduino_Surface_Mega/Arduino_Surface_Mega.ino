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
  
String serialString;

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
    ReadSerialStream();
    ProcessMotorCommand(serialString);
  } 
}

/// Start of packet character: .
/// End of packet character: *
void ReadSerialStream(void)
{ 
  String inputString;
  while (Serial.available() > 0)
  {
    
    char c = (char)Serial.read(); // Incoming byte
    
    if (c == '*') // If the char is the end of the packet, stop here and process it
    { 
      serialString = inputString;
      return; // leave so main loop can process it
    }
    
    if (c == '.')  // beginning of packet
    {
      inputString = ""; //clear the current buffer; we're starting a new packet
    }
    else
      inputString += c; // if it's not a start or finish character, then add it to the buffer
      
    if (inputString.length() > MAXPACKETLEN) // the packet is longer than any packet we'd expect, throw it out
    {
      inputString = ""; // clear the input string, because obviously this is invalid
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

