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

void printDebugHeader()
{

#ifdef DEBUG

  Serial.print("iteration,rumTime,loopTime,");

#endif

#ifdef DEBUG_MPU

    Serial.println("gRate_Y,euler_psi,gRate_P,euler_theta,gRate_R,euler_phi,");

#endif
    
#ifdef DEBUG_KALMAN

    Serial.println("P_00_Y,P_01_Y,P_10_Y,P_11_Y,k_0_Y,K_1_Y,Q-angle_Y,Q_gyro_Y,R_angle_Y,bias_Y,fusedAngle_Y,P_00_P\tP_01_P,P_10_P,P_11_P,k_0_P,K_1_P,Q-angle_P,Q_gyro_P,R_angle_P,bias_P,fusedAngle_P,P_00_R,P_01_R,P_10_R,P_11_R,k_0_R,K_1_R,Q-angle_R,Q_gyro_R,R_angle_R,bias_R,fusedAngle_R,");

#endif

#ifdef DEBUG_CONTROLER

    Serial.println("P_coef,I_corf,D_coef,PID_coef,euler_1,com_ang,error,P_vector[0],I_vector[1],D_vector[2],pid_sum,Dtorque_by_fall,motorArray[0],motorArray[1],");

#endif

#ifdef DEBUG_MOTORS

  Serial.print("MotorA_value,MotorA_brk,MotorA_dir,MotorB_value,MotorB_brk,MotorB_dir,");
    
#endif

}


