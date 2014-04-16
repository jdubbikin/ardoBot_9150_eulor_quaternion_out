void doEncoder1A()
{
  // Look for low to high on encoder 1A
  if(digitalRead(ENCODER_1A) == HIGH)
  {
    if(digitalRead(ENCODER_1B) == LOW)
    {
      encoder1Pos += 1;
    }
    else
    {
      encoder1Pos -= 1;
    }
  }
  else
  {
    if(digitalRead(ENCODER_1B) == HIGH)
    {
      encoder1Pos += 1;
    }
    else
    {
      encoder1Pos -= 1;
    }
  }
  //Serial.print(digitalRead(30));Serial.print("\t");
  //Serial.print(digitalRead(31));Serial.print("\t");
  //Serial.print(digitalRead(32));Serial.print("\t");
  //Serial.print(digitalRead(33));Serial.print("\t");
  //Serial.print(encoder1Pos, DEC);Serial.print("\t");
  //Serial.print(encoder2Pos, DEC);Serial.print("\t");
  //Serial.println();
}


void doEncoder1B()
{
  // Look for low to high on encoder 1A
  if(digitalRead(ENCODER_1B) == HIGH)
  {
    if(digitalRead(ENCODER_1A) == HIGH)
    {
      encoder1Pos += 1;
    }
    else
    {
      encoder1Pos -= 1;
    }
  }
  else
  {
    if(digitalRead(ENCODER_1A) == LOW)
    {
      encoder1Pos += 1;
    }
    else
    {
      encoder1Pos -= 1;
    }
  }
  //Serial.print(digitalRead(30));Serial.print("\t");
  //Serial.print(digitalRead(31));Serial.print("\t");
  //Serial.print(digitalRead(32));Serial.print("\t");
  //Serial.print(digitalRead(33));Serial.print("\t");
  //Serial.print(encoder1Pos, DEC);Serial.print("\t");
  //Serial.print(encoder2Pos, DEC);Serial.print("\t");
  //Serial.println();
  // Serial.println(encoder1Pos, DEC);      // debug code comment out for efficiancy. 
}


void doEncoder2A()
{
  // Look for low to high on encoder 1A
  if(digitalRead(ENCODER_2A) == HIGH)
  {
    if(digitalRead(ENCODER_2B) == LOW)
    {
      encoder2Pos -= 1;
    }
    else
    {
      encoder2Pos += 1;
    }
  }
  else
  {
    if(digitalRead(ENCODER_2B) == HIGH)
    {
      encoder2Pos -= 1;
    }
    else
    {
      encoder2Pos += 1;
    }
  }
  //Serial.print(digitalRead(30));Serial.print("\t");
  //Serial.print(digitalRead(31));Serial.print("\t");
  //Serial.print(digitalRead(32));Serial.print("\t");
  //Serial.print(digitalRead(33));Serial.print("\t");
  //Serial.print(encoder1Pos, DEC);Serial.print("\t");
  //Serial.print(encoder2Pos, DEC);Serial.print("\t");
  //Serial.println();
  // Serial.println(encoder2Pos, DEC);      // debug code comment out for efficiancy. 
}


void doEncoder2B()
{
  // Look for low to high on encoder 1A
  if(digitalRead(ENCODER_2B) == HIGH)
  {
    if(digitalRead(ENCODER_2A) == HIGH)
    {
      encoder2Pos -= 1;
    }
    else
    {
      encoder2Pos += 1;
    }
  }
  else
  {
    if(digitalRead(ENCODER_2A) == LOW)
    {
      encoder2Pos -= 1;
    }
    else
    {
      encoder2Pos += 1;
    }
  }
  //Serial.print(digitalRead(30));Serial.print("\t");
  //Serial.print(digitalRead(31));Serial.print("\t");
  //Serial.print(digitalRead(32));Serial.print("\t");
  //Serial.print(digitalRead(33));Serial.print("\t");
  //Serial.print(encoder1Pos, DEC);Serial.print("\t");
  //Serial.print(encoder2Pos, DEC);Serial.print("\t");
  //Serial.println();
  // Serial.println(encoder2Pos, DEC);      // debug code comment out for efficiancy. 
}


void encoderSetup()
{
   pinMode(ENCODER_1A, INPUT);
   pinMode(ENCODER_1B, INPUT);
   pinMode(ENCODER_2A, INPUT);
   pinMode(ENCODER_2B, INPUT);
   
   //pinMode(ENCODER_1_LED, OUTPUT);
   //pinMode(ENCODER_2_LED, OUTPUT);
   
  attachInterrupt(30, doEncoder1A, CHANGE);
  attachInterrupt(31, doEncoder1B, CHANGE);
  attachInterrupt(32, doEncoder2A, CHANGE);
  attachInterrupt(33, doEncoder2B, CHANGE);
  
  encoder1Pos = 0;
  encoder2Pos = 0;
}
