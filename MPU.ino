void initMPU()
{
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  
  digitalWrite(MPU_INIT_PIN, LOW);
  //bool standby = 0;
  while( digitalRead(MPU_INIT_PIN) == HIGH )
  {
    digitalWrite(FAIL_PIN, HIGH);
    Serial.println("Hit go button to begin DMP programming: ");
    delay(100);
    digitalWrite(FAIL_PIN, LOW);
    delay(100);
  }
  digitalWrite(GO_PIN, HIGH);
  digitalWrite(FAIL_PIN, HIGH);

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 40)..."));
        attachInterrupt(40, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        digitalWrite(GO_PIN, HIGH);
        digitalWrite(FAIL_PIN, LOW);
        
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else 
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        digitalWrite(FAIL_PIN, HIGH);
        digitalWrite(GO_PIN, LOW);
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  
}

void getMPU_Interupt()
{
  // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
}

void readFIFOpacket()
{
  // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        
#ifdef DEBUG
        Serial.println();
        Serial.println("FIFO overflow!");
        Serial.println();
#endif

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) 
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

    }
}

/* ================================================================================================ *
 | Default MotionApps v4.1 48-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][MAG X ][MAG Y ][MAG Z ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ] |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41  42  43  44  45  46  47  |
 * ================================================================================================ */
void getMPUData()
{
  // display quaternion values in easy matrix form: w x y z
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  //mpu.dmpGetGravity(&gravity, &q);
  //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetGyro(gRate, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  
#ifdef DEBUG_MPU
    
    for(int i = 0; i < 3; i++)
    {
      Serial.print(gRate[i]);Serial.print(",");
      Serial.print(euler[i]);Serial.print(",");
    }
    
#endif
  
}

