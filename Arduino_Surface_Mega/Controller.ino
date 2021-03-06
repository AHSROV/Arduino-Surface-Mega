int motorInt[6] = {};
float motorUpBack, motorUpFront, motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
void ProcessController()
{ 
  if(Xbox.getButtonClick(L1))
  {
    if(motorPercentage > .1)
    {
      motorPercentage -= .1;
    }
  }  
  if(Xbox.getButtonClick(R1))
  {
    if(motorPercentage < 1)
    {
      motorPercentage += .1;
    }
  }  
  if(Xbox.getButtonClick(START))
  {
    controlSet = 1;
  }
  if(Xbox.getButtonClick(BACK))
  {
    controlSet = 0;
  }
  
  if(controlSet == 0)
    ControllerForward();
  if(controlSet == 1)
    ControllerBack();  
  
  motorInt[5] = (motorUpFront * 100) * .8 * motorPercentage;
  motorInt[4] = (motorUpBack * 100) * .8 * motorPercentage;
  motorInt[0] = (motorFrontLeft * 100) * .8 * motorPercentage;
  motorInt[1] = (motorFrontRight * 100) * .8 * motorPercentage;
  motorInt[2] = (motorBackLeft * 100) * .8 * motorPercentage;
  motorInt[3] = (motorBackRight * 100) * .8 * motorPercentage;
  
  for(int i = 0; i < 6; i++)
  {
    bool negative;
    if(motorInt[i] < 0)
      negative = true;
    else
      negative = false;

    SetMotor(abs(motorInt[i]), i, !negative);
  }
}

void ControllerForward() 
{
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
}

void ControllerBack()
{
  if(Xbox.Xbox360Connected)
  {
    // Up Down controlled by triggers
    motorUpBack = NormalLongToFloat(Xbox.getButtonPress(L2) * 128) - NormalLongToFloat(Xbox.getButtonPress(R2)*128);
    motorUpFront = NormalLongToFloat(Xbox.getButtonPress(L2) * 128) - NormalLongToFloat(Xbox.getButtonPress(R2)*128);
    
    motorFrontLeft = NormalLongToFloat(Xbox.getAnalogHat(LeftHatX));
    motorFrontRight = -NormalLongToFloat(Xbox.getAnalogHat(LeftHatX));
    motorBackLeft = NormalLongToFloat(Xbox.getAnalogHat(LeftHatX));
    motorBackRight = -NormalLongToFloat(Xbox.getAnalogHat(LeftHatX));
    
    //forward and backward movement it controlled by the left y-axis
    motorFrontRight += NormalLongToFloat(Xbox.getAnalogHat(LeftHatY));
    motorFrontLeft  += NormalLongToFloat(Xbox.getAnalogHat(LeftHatY));
    motorBackLeft   += -NormalLongToFloat(Xbox.getAnalogHat(LeftHatY));
    motorBackRight  += -NormalLongToFloat(Xbox.getAnalogHat(LeftHatY));

    //rotation is controlled by the right thumbstick x-axis
    motorFrontLeft  += -NormalLongToFloat(Xbox.getAnalogHat(RightHatX));
    motorFrontRight += NormalLongToFloat(Xbox.getAnalogHat(RightHatX));
    motorBackLeft   += NormalLongToFloat(Xbox.getAnalogHat(RightHatX));
    motorBackRight  += -NormalLongToFloat(Xbox.getAnalogHat(RightHatX));

    //Pitch is controlled by the right thumbstick y-axis
    motorUpBack  += -NormalLongToFloat(Xbox.getAnalogHat(RightHatY));
    motorUpFront += NormalLongToFloat(Xbox.getAnalogHat(RightHatY));
    
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
