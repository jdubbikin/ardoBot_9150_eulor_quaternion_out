/*
// { P_00, P_01, P_10, P_11, k_0, K_1, Q-angle, Q_gyro, R_angle, n_bias, fusedAngle, newRate, newAngle }
//    0      1    2      3    4    5    6        7          8      9        12          10      11
  void kalmanCalculate(void) 
  { 
      
    /*
    
    dt = float(looptime)/1000;
    x_angle += dt * (newRate - x_bias);
    P_00 += dt * (P_10 + P_01) + Q_angle * dt;
    P_01 += dt * P_11;
    P_10 += dt * P_11;
    P_11 += Q_gyro * dt;

    y = newAngle - x_angle;
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;

    x_angle +=  K_0 * y;
    x_bias  +=  K_1 * y;
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;

    return x_angle;
    // dataFusion[i][j] =
    // { P_00, P_01, P_10, P_11, k_0, K_1, Q-angle, Q_gyro, R_angle, bias, fusedAngle, //newRate, //newAngle }
    //    0      1    2      3    4    5     6        7        8      9        10        11        12
    
    *//*
    
    for(int i = 0; i < 3; i++)
    {

      dataFusion[10][i] += deltaT * (gRate[i] - dataFusion[9][i]);
      dataFusion[0][i] += - deltaT * (dataFusion[2][i] + dataFusion[1][i]) + dataFusion[6][i] * deltaT;
      dataFusion[1][i] += - deltaT * dataFusion[3][i];
      dataFusion[2][i] += - deltaT * dataFusion[3][i];
      dataFusion[3][i] += + dataFusion[7][i] * deltaT;

      y = euler[i] - dataFusion[10][i];
      S = dataFusion[0][i] + dataFusion[8][i];
      
      if(S == 0){S = 1 / 100000000;}  // Do not devide by 0.
      
      dataFusion[4][i] = dataFusion[0][i] / S;
      dataFusion[5][i] = dataFusion[2][i] / S;

      dataFusion[10][i] +=  dataFusion[4][i] * y;
      dataFusion[9][i]  +=  dataFusion[5][i] * y;
      dataFusion[0][i] -= dataFusion[4][i] * dataFusion[0][i];
      dataFusion[1][i] -= dataFusion[4][i] * dataFusion[1][i];
      dataFusion[2][i] -= dataFusion[5][i] * dataFusion[0][i];
      dataFusion[3][i] -= dataFusion[5][i] * dataFusion[1][i]; 
    } 
      
  #ifdef DEBUG_KALMAN
    Serial.print(iteration);Serial.print(",");
    Serial.print(deltaT); Serial.print(",");
    
    for(int i = 0; i < 3; i++)
    {
      Serial.print(gRate[i]);Serial.print(",");
      Serial.print(euler[i]);Serial.print(",");
    }
    
    for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < 11; j++)
      {
        Serial.print(dataFusion[i][j]); Serial.print(",");
      }
    }
    
  #endif
  }
 */
