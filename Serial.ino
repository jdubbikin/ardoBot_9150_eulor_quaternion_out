void printOutput()
{
#ifdef RUNTIME
  Serial.print(q.w);Serial.print("\t");
  Serial.print(q.x);Serial.print("\t");
  Serial.print(q.y);Serial.print("\t");
  Serial.print(q.z);Serial.print("\t");

  // display Euler angles in degree
  Serial.print(euler[0]);Serial.print("\t");
  Serial.print(euler[1]);Serial.print("\t");
  Serial.print(euler[2]);Serial.print("\t");
  
  //Serial.print(dataFusion[10][0]);Serial.print("\t");
  //Serial.print(dataFusion[10][1]);Serial.print("\t");
  //Serial.print(dataFusion[10][2]);Serial.print("\t");
#endif
  Serial.println();
  
}

void checkInputData()
{
  if (Serial.available()) 
  {
    switch (Serial.read()) 
    {
      case ',':
      case '<':
        pidCoefVector[3] -= 0.01;
        return;
        
      case '.':
      case '>':
        pidCoefVector[3] += 0.01;
        return;
        
      case 'l':
      case 'L':
      
        printOutput();
        return;
        
      //default:
      //readSensor();
    }
  } 
}
