 
void initMotorControler()
{
  pinMode(A_dir, OUTPUT);  // This sets the direction pin as an output.
  pinMode(A_spd, OUTPUT);  // This sets the brake pin as an output.
  pinMode(A_brk, OUTPUT);  // This sets the brake pin as an output.
  
  pinMode(B_dir, OUTPUT);  // This sets the direction pin as an output.
  pinMode(B_spd, OUTPUT);  // This sets the brake pin as an output.
  pinMode(B_brk, OUTPUT);  // This sets the brake pin as an output.
  
  Serial.println("Motor controler: configured and active");
}

void motorControl1(long value)
{
  value = constrain(value, -255, 255);
   
  if(value < 0)
  {
    digitalWrite(A_brk, LOW);
    digitalWrite(A_dir, HIGH);
    analogWrite(A_spd, abs(value));
    
#ifdef DEBUG_MOTORS
    Serial.print(value);Serial.print("\t");
    Serial.print(digitalRead(A_brk));Serial.print("\t");
    Serial.print(digitalRead(A_dir));Serial.print("\t");
#endif
    
  }
  
  if(value > 0)
  {
    digitalWrite(A_brk, LOW);
    digitalWrite(A_dir, LOW);
    analogWrite(A_spd, abs(value));
    
#ifdef DEBUG_MOTORS
    Serial.print(value);Serial.print("\t");
    Serial.print(digitalRead(A_brk));Serial.print("\t");
    Serial.print(digitalRead(A_dir));Serial.print("\t");
#endif
    
  }
  if(value == 0)
  {
    digitalWrite(A_brk, HIGH);
    
#ifdef DEBUG_MOTORS
    Serial.print(value);Serial.print("\t");
    Serial.print(digitalRead(A_brk));Serial.print("\t");
    Serial.print(digitalRead(A_dir));Serial.print("\t");
#endif

  }
  
}

void motorControl2(long value)
{
  
  if(value < 0)
  {
    digitalWrite(B_brk, LOW);
    digitalWrite(B_dir, HIGH);
    analogWrite( B_spd, abs(value) );
    
#ifdef DEBUG_MOTORS
    Serial.print(value);Serial.print("\t");
    Serial.print(digitalRead(B_brk));Serial.print("\t");
    Serial.print(digitalRead(B_dir));Serial.print("\t");
#endif
    
  }
  else if(value > 0)
  {
    digitalWrite(B_brk, LOW);
    digitalWrite(B_dir, LOW);
    analogWrite( B_spd, abs(value) );
    
#ifdef DEBUG_MOTORS
    Serial.print(value);Serial.print("\t");
    Serial.print(digitalRead(B_brk));Serial.print("\t");
    Serial.print(digitalRead(B_dir));Serial.print("\t");
#endif
    
  }
  else
  {
    digitalWrite(B_brk, HIGH);
    
#ifdef DEBUG_MOTORS
    Serial.print(value);Serial.print("\t");
    Serial.print(digitalRead(B_brk));Serial.print("\t");
    Serial.print(digitalRead(B_dir));Serial.print("\t");
#endif
    
  }
}

