// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define DEBUG
#define DEBUG_CONTROLER
// #define DEBUG_KALMAN
// #define DEBUG_MOTORS
// #define RUNTIME

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
#define BALLANCE_TOLLERANCE  0.01
#define TORQUE_GRAVITY       0.985125
#define TORQUE_MOTOR_A       0.68614
#define TORQUE_MOTOR_B       0.470496
#define E                    2.7182818285
#define MOTOR_A_GAIN         3
#define MOTOR_B_GAIN         3
#define A1                   53.26  
#define B1                   3411
// #define C1                   0.0001071
#define A2                   47.40
#define B2                   6223
// #define C2                   0.0005630

MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #40 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 40.
 * ========================================================================= */

#define OUTPUT_READABLE_QUATERNION
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t gRate[3];       // [x, y, z]            get the rotation rates from the gyro 

// Kalmen veriables. 
float dataFusion[13][2];
float y, S;

// command veriables. 
 float command_angle = 0;
 
// controler veriables
float angle_error = 0;
float old_angle_error = 0;
float pidSum = 0;
float pidVector[] = {0.0, 0.0, 0.0};
float pidCoefVector[] = { 0.0, 0.0, 0.0, 1.0 };   // { P_coef, I_coef, D_coef, PID_coef }

// Encoder declaratins;
 #define ENCODER_1A 30
 #define ENCODER_1B 31
 #define ENCODER_2A 32
 #define ENCODER_2B 33

 volatile int encoder1Pos;
 volatile int encoder2Pos;
 
 // Declare the Motor veriables. 
 int motorArray[] = { 0, 0 };
 
 const int A_dir = 12;
 const int A_spd = 3;
 const int A_brk = 9;
 const int A_amp_sence = 0;
 
 const int B_dir = 13;
 const int B_spd = 11;
 const int B_brk = 8;
 const int B_amp_sence = 0;

 // Time veriables. 
 double deltaT, initalTime, currentTime, oldTime = 0;
 int iteration = 0;

// INTERRUPT DETECTION ROUTINE           
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() 
{
    mpuInterrupt = true;
}

// INITIAL SETUP   
void setup() 
{
    // initialize serial communication
    Serial.begin(115200);
    Wire.begin();
    
    for(int i = 0; i < 3; i++)
    {
      dataFusion[6][i] = 0.001;
      dataFusion[7][i] = 0.003;
      dataFusion[8][i] = 0.03;
      
#ifdef DEBUG_KALMAN

        Serial.print(dataFusion[6][i]);Serial.print("\t");
        Serial.print(dataFusion[7][i]);Serial.print("\t");
        Serial.print(dataFusion[8][i]);Serial.print("\t");
        Serial.println();  
        
#endif
      
    }

    initMPU();
    
    initMotorControler();
    
    // encoderSetup();
    
#ifdef DEBUG

  Serial.print("iteration\trumTime\tloopTime\t");

#endif
    
#ifdef DEBUG_KALMAN

    Serial.println("gRate_Y\teuler_psi\tgRate_P\teuler_theta\tgRate_R\teuler_phi\tP_00_Y\tP_01_Y\tP_10_Y\tP_11_Y\tk_0_Y\tK_1_Y\tQ-angle_Y\tQ_gyro_Y\tR_angle_Y\tbias_Y\tfusedAngle_Y\tP_00_P\tP_01_P\tP_10_P\tP_11_P\tk_0_P\tK_1_P\tQ-angle_P\tQ_gyro_P\tR_angle_P\tbias_P\tfusedAngle_P\tP_00_R\tP_01_R\tP_10_R\tP_11_R\tk_0_R\tK_1_R\tQ-angle_R\tQ_gyro_R\tR_angle_R\tbias_R\tfusedAngle_R ");

#endif

#ifdef DEBUG_CONTROLER

    Serial.println("P_coef\tI_corf\tD_coef\tPID_coef\teuler_1\tcom_ang\terror\tP_vector[0]\tI_vector[1]\tD_vector[2]\tpid_sum\tDtorque_by_fall\tmotorArray[0]\tmotorArray[1]\t");

#endif

#ifdef DEBUG_MOTORS

  Serial.print("MotorA_value\tMotorA_brk\tMotorA_dir\tMotorB_value\tMotorB_brk\tMotorB_dir\t");
    
#endif

    initalTime = millis();
    
}

// MAIN PROGRAM LOOP
void loop() 
{
  currentTime = millis() - initalTime;
  deltaT = currentTime - oldTime;
  oldTime = currentTime;
  
  // checkInputData();
  
#ifdef DEBUG

  Serial.print(iteration);Serial.print("\t");
  Serial.print(currentTime);Serial.print("\t");
  Serial.print(deltaT);Serial.print("\t");
  
#endif
  
  analogReadResolution(12);
  
  pidCoefVector[0] = analogRead(7) / 4095 ;
  pidCoefVector[1] = analogRead(8) / 4095 ;
  pidCoefVector[2] = analogRead(9) / 4095 ;
  
#ifdef DEBUG_CONTROLER

      Serial.print( pidCoefVector[0] ); Serial.print("\t");
      Serial.print( pidCoefVector[1] ); Serial.print("\t");
      Serial.print( pidCoefVector[2] ); Serial.print("\t");
      Serial.print( pidCoefVector[3] ); Serial.print("\t");
  
#endif
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) 
  {
      
   // other program behavior stuff here
   // Serial.println("Doing some stuff!!!!!");
        
     
  }

  getMPU_Interupt();

  readFIFOpacket();
    
  getMPUData();
  
  // kalmanCalculate(); 
  
  // Complie data send to pid 
  pidControler();
  
#ifdef DEBUG

  iteration++;
  Serial.println();
  
#endif

}


