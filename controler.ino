
// Torque = mass * Gravity * Distance. 
// Unit mass = 0.804kg center of mass = 0.125mm = 0.125m from axel in Z direction
// => Torque = 0.804kg * 9.802 m/s^2 * 0.125m * sin(theta) = 0.985125 N/m at 90°
// Motor  A force = (0.68614 N at MAX PWM
// Motor  B force = (0.470496 N at MAX PWM
// Wheel radius = 21mm = 0.021 m. 
// Wheel A max torque = 0.68614N * 0.021m = 0.00144 N m
// Wheel B max torque = 0.470496N * 0.021m = 0.0098804 N m 
// Motor A line fit Equation. Torque = 73.10*e^(0.009290(PWM))-74.77
// Solved for PWM = 107.643*ln[0.0136799(74.77 + T_body)]
// Motor B linr fit Equation. Torque = 48.50*e^(0.009225(PWM))-59.79
// Solved for PWM = 108.401*ln[0.0206186(48.50 + T_bodey)]
// natural exponent e = 2.7182818285
// pow(base, exponent)
// pwmA = 107.643 * pow( E, 0.0136799 * (74.77 + torque_body) );
// pwmB = 108.401 * pow( E, 0.0206186 * (48.50 + torque_body) );
//
// New Fit experiment reasult. 
// Motor A
// Quadratic Fit Ax^2 + Bx + C = PWM
// A = 3.814E-07
// B = -1224E-05
// C = 0.0001071
// Motor B
// A = 4.243E-07
// B = -3.190E-05
// C = 0.0005630

// With x beign required torque and y the PWM output. A curve fit 
// to A*ln(B*x) gives
// A1 = 53.26
// B1 = 3411
// A2 = 47.40
// B2 = 6223

// Needed tourque / 2 for each motor. 
      
void pidControler(void)
{  
  if(isnan(euler[1])){euler[1] = 0;}
  
  angle_error = command_angle - euler[1];   
  
  pidVector[0] = angle_error * pidCoefVector[0];
  pidVector[1] += angle_error * pidCoefVector[1];
  pidVector[2] = ( angle_error - old_angle_error ) * pidCoefVector[2];
  
  pidSum = pidVector[0] + pidVector[1] + pidVector[2];
  
  old_angle_error = angle_error;
  
#ifdef DEBUG_CONTROLER

   Serial.print( euler[1] );      Serial.print("\t");
   Serial.print( command_angle ); Serial.print("\t");
   Serial.print( angle_error );   Serial.print("\t");
   Serial.print( pidVector[0] );  Serial.print("\t");
   Serial.print( pidVector[1] );  Serial.print("\t");
   Serial.print( pidVector[2] );  Serial.print("\t");
   Serial.print( pidSum );        Serial.print("\t");
  
#endif 

  pid_motorOutput( euler[1] + pidSum * pidCoefVector[3] );
}

void pid_motorOutput(float angle)
{ 
   
  float fallTorque = (TORQUE_GRAVITY * sin( angle ) );
   
  //motorArray[0] = 254 - (227.1 * pow( E, -68.14 * torque_by_fall) );
  //motorArray[1] = 250 - (252.8 * pow( E, -43.38 * torque_by_fall) );
  
  motorArray[0] = A1 * log( B1 * abs(fallTorque) ) / 2;
  motorArray[1] = A2 * log( B2 * abs(fallTorque) ) / 2;
  
  if( euler[1] < 0)
  {
    motorArray[0] = -motorArray[0];
    motorArray[1] = -motorArray[1];
  }

  
  if(isnan(motorArray[0])){motorArray[0] = 0;}
  if(isnan(motorArray[1])){motorArray[1] = 0;}
  
  motorControl1( - motorArray[0] * MOTOR_A_GAIN );
  motorControl2( - motorArray[1] * MOTOR_B_GAIN );
 
#ifdef DEBUG_CONTROLER

  Serial.print( fallTorque ); Serial.print("\t");
  Serial.print( motorArray[0] ); Serial.print("\t");
  Serial.print( motorArray[1] ); Serial.print("\t");
  
#endif

}
