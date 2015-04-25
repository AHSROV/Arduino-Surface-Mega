void ProcessController()
{
  float motorUpBack, motorUpFront, motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
  
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
  
  motorInt[5] = (motorUpFront * 100) * .8;
  motorInt[4] = (motorUpBack * 100) * .8;
  motorInt[0] = (motorFrontLeft * 100) * .8;
  motorInt[1] = (motorFrontRight * 100) * .8;
  motorInt[2] = (motorBackLeft * 100) * .8;
  motorInt[3] = (motorBackRight * 100) * .8;
  
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
