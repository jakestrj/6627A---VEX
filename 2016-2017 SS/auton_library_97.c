// Wrap code with definition
#ifndef  _AUTON_LIBRARY
#define  _AUTON_LIBRARY
#define ACY

#define abs(X) ((X < 0) ? -1 * X : X)
#define neg(X) ((X > 0) ? -1 * X : X)
#define mLIFTR_1 liftRY
#define mLIFTR_2 liftR
#define mLIFTL_1 liftLY
#define mLIFTL_2 liftL
#define mCLAWL clawL
#define mCLAWR clawR
#define mDRIVER_1 driveR1
#define mDRIVER_2 driveR2
#define mDRIVEL_1 driveL1
#define mDRIVEL_2 driveL2
#define mDRIVEL_3 driveL3
#define mDRIVER_3 driveR3
#define sPOTL pot_lift
#define sBUTL but_lift
#define sENCL quad_driveL
#define sENCR quad_driveR

#define resetEncLeft()  SensorValue[sENCL]=0
#define resetEncRight() SensorValue[sENCR]=0

#define getEncLeft()   abs(SensorValue[sENCL])
#define getEncRight() abs(SensorValue[sENCR])

#define getInchesLeft()  (float) ( (float) SensorValue[sENCL]/29.2)
#define getInchesRight() (float) ( (float)-SensorValue[sENCR]/29.2)

#define getSensorPotLift() (float) abs(SensorValue[sPOTL]-200)

//functional shorteners--------------------------------------------------------------------------------------------------------------
void pwrClaw(int pwr){motor[mCLAWL]=motor[mCLAWR]=pwr;}
void pwrDrive(int speed){motor[mDRIVEL_1] = motor[mDRIVEL_2] = motor[mDRIVEL_3] = speed; motor[mDRIVER_1] = motor[mDRIVER_2] = motor[mDRIVER_3] = speed;}
void pwrDriveRight (int speed){motor[mDRIVER_1] = motor[mDRIVER_2] = motor[mDRIVER_3] = speed;}
void pwrDriveLeft (int speed){motor[mDRIVEL_1] = motor[mDRIVEL_2] = motor[mDRIVEL_3] = speed;}
//-----------------------------------------------------------------------------------------------------------------------------------

#ifdef ACN
void autonTurn( int degrees, int speed ) {

	clearDebugStream();

	float inches;
	float inchesL=0;
	float inchesR=0;
	int timeout=0;

	inches =  ( ((float)abs(degrees)/(float)360) * (float)/*44*/22 );

	if (degrees < 0) {
		speed = -speed;
	}

    writeDebugStream( "AUTON TURN| inches: %d | speed: %d\n", inches, speed );

	resetEncLeft();
	resetEncRight();

	// Start turn
    pwrDriveLeft( -speed );
    pwrDriveRight( speed );

	while( true ) {

		if (inchesL == getInchesLeft() && inchesR == getInchesRight() ) {
			timeout++;
			if( timeout > 30 ) {
				break;
			}
		}

		inchesL = getInchesLeft();
		inchesR = getInchesRight();



		if( abs(inchesL) > inches || abs(inchesR) > inches ) {
			break;
		}

		//writeDebugStream( "AT| lastInchesL: %d | lastInchesR: %d | currentInchesL: %d | currentInchesR: %d\n", inchesL, inchesR, getInchesLeft(), getInchesRight() );
		wait1Msec(20);
	}

    // Reverse motors for a moment to stop coast
    pwrDriveLeft( -speed/2 );
    pwrDriveRight( speed/2 );

    wait1Msec( abs(speed*1.5) );

    // Stop motors
    pwrDrive(0);

}

void autonDrive( int inches, int speed ) {

	clearDebugStream();

	float inchesL=0;
	float inchesR=0;
	int timeout=0;

	resetEncLeft();
	resetEncRight();

	writeDebugStream( "AUTON DRIVE| inches: %d | speed: %d\n", inches, speed );

    // Start to motors
    pwrDriveLeft( speed );
    pwrDriveRight( speed );

	while( true ) {

		if (inchesL == getInchesLeft() && inchesR == getInchesRight() ) {
			timeout++;
			if( timeout > 30 ) {
				break;
			}
		}

		inchesL = getInchesLeft();
		inchesR = getInchesRight();

		// Called to apply power and correct offset error
	    if(abs(inchesR)>abs(inchesL))
	    {
	      pwrDriveRight(speed-5);
	      pwrDriveLeft(speed);
	    }
	    else if(abs(inchesR)<abs(inchesL))
	    {
	      pwrDriveRight(speed);
	      pwrDriveLeft(speed-5);
	    }
	    else
	    {
	      pwrDrive(speed);
	    }


		if( abs(inchesL) > inches || abs(inchesR) > inches ) {
			break;
		}

		//writeDebugStream( "AD| lastInchesL: %d | lastInchesR: %d | currentInchesL: %d | currentInchesR: %d\n", inchesL, inchesR, getInchesLeft(), getInchesRight() );
		wait1Msec(20);
	}

	// Reverse motors to stop coast
    pwrDriveLeft( -speed/2 );
    pwrDriveRight( -speed/2 );

    wait1Msec( abs(speed/2) );

    // Stop motors
    pwrDrive(0);

}
#endif

/////////////////////////////////////////////////////////////////

#ifdef ACY
typedef enum {Left, Right} Direction;
#define GearRatioAdjustment 1.0             		 //adjust to gear ratio
#define Four_Inch_Diameter (3.98/GearRatioAdjustment)//diameter of the 4 inch wheels
#define circumference (3.14 * 3.98)					 //calculating the circumference (of the 4 inch wheels)
#define Degrees2Inches (3.14/GearRatioAdjustment)    //degrees in an inch
#define GearRatio (1.0)

long LastEncoderValueLeft = -1;
long LastEncoderValueRight = -1;

void autonTurn(int degrees, int speed ,bool driftCorrection = true)
{
	clearTimer(T4);
	wait1Msec(100);
	int EncValue;
	float distance;
	tMotor MotorValuePos1;
	tMotor MotorValuePos2;
	tMotor MotorValuePos3;
	tMotor MotorValueNeg1;
	tMotor MotorValueNeg2;
	tMotor MotorValueNeg3;
	Direction TurnDirection;
	int CurrEncValueLeft;            //current value of the left encoder
	int CurrEncValueRight;           //current value of the right encoder
	int EncValueMax = 0;             //maximum encoder value recieved
	bool EncValueExceeded = false;   //state if the encoder value has been exceeded
	bool exitLoop = false;           //state determining whether the loop should be exited

	resetEncRight(); //reset encoders
	resetEncLeft();

	distance = ((float) degrees / Degrees2Inches) / 4.0 / 2.0; // /2.0 added
	EncValue = 1120.0 * distance / circumference;  //Converts degrees into encoder value

	if(degrees < 0) TurnDirection = Left
		else TurnDirection = Right

	MotorValueNeg1 = mDRIVEL_1;
	MotorValueNeg2 = mDRIVEL_2;
	MotorValueNeg3 = mDRIVEL_3;
	MotorValuePos1 = mDRIVER_1;
	MotorValuePos2 = mDRIVER_2;
	MotorValuePos3 = mDRIVER_3;

	//Reset and initialize values
	EncValue = EncValue*(GearRatio);
	nMotorEncoder[MotorValuePos1] = 0;
	nMotorEncoder[MotorValueNeg1] = 0;

	CurrEncValueLeft = getEncLeft();
	CurrEncValueRight = getEncRight();


	//Determine configuration for different turn directions
	if (TurnDirection==Left)
	{ //Left wheel will move backwards, front forwards
		motor[MotorValuePos1] = motor[MotorValuePos2] = motor[MotorValuePos3] = -speed;
		motor[MotorValueNeg1] = motor[MotorValueNeg2] = motor[MotorValueNeg2] = speed;
	}
	else
	{ //Left wheel will move forwards, front backwards
		motor[MotorValuePos1] = motor[MotorValuePos2] = speed;
		motor[MotorValueNeg1] = motor[MotorValueNeg2] = -speed;
	}

	while(!exitLoop)
	{ //while distance not reached
		CurrEncValueLeft = getEncLeft();
		CurrEncValueRight = getEncRight();

		if(abs(CurrEncValueLeft) > abs(EncValue))
		{  // if the distance is reached

			if(EncValueExceeded &&(EncValueMax != CurrEncValueLeft))
				exitLoop = true;
			else
			{
				EncValueMax = CurrEncValueLeft;
				EncValueExceeded = true;
			}
		} // end if the distance is reached
		else
			EncValueExceeded = false;
	} //while distance not matched
	if(driftCorrection){
		if (TurnDirection==Left)
		{
			pwrDriveLeft( -speed/2 );
	    	pwrDriveRight( speed/2 );
		}
		else
		{
			pwrDriveLeft( speed/2 );
	    	pwrDriveRight( -speed/2 );
		}

		// Reverse motors for a moment to stop coast

	    wait1Msec( abs(speed*1.5) );
    }

    // Stop motors
    pwrDrive(0);
	//return(Exit);
} // end TurnInPlace2


void autonDrive(float distance, int speed, bool driftCorrection = true, int correction = 5)
{
	long EncValue;                 // # of encoder ticks corresponding to distance
	long CurrEncValueLeft;         //current value of the left encoder
	long CurrEncValueRight;        //current value of the right encoder
	float Sensitivity = 0.15;       //multiplying factor of adjustion
	int RightSpeed;                //calculated speed of right motor
	int LeftSpeed;                 //calculated speed of left motor
	long EncValueMax = 0;          //maximum encoder value recieved
	bool EncValueExceeded = false; //state if the encoder value has been exceeded
	bool exitLoop = false;         //state determining whether the loop should be exited
	int MinSpeed = 50;
	if(MinSpeed>speed)
		MinSpeed = speed;

	clearTimer(T4);

	resetEncRight(); //reset encoders
	resetEncLeft();

	wait1Msec(100);

	EncValue = 1120.0 * distance / circumference / 3.5 // /3.0 added

	CurrEncValueLeft = 0;
	CurrEncValueRight = 0;

	while(!exitLoop)
	{ //while distance not matched
		// nxtDisplayTextLine(4, "Right Encoder %.1d", nMotorEncoder[Right1]);
		// nxtDisplayTextLine(5, "Left Encoder %.1d", nMotorEncoder[Left1]);

		//If the wheels are not turning, stop them to prevent motor burnouts
		if(time1[T4] > 300)
		{

			clearTimer(T4);
			if((CurrEncValueLeft == LastEncoderValueLeft) || (CurrEncValueRight == LastEncoderValueRight))
			{
				exitLoop = true;
			}
			else
			{
				LastEncoderValueLeft = CurrEncValueLeft;
				LastEncoderValueRight = CurrEncValueRight;
			}
		}

		//Drive straight, adjusting for natural drift of the robot
		if(abs(CurrEncValueLeft) > abs(CurrEncValueRight))
		{
			LeftSpeed = (speed - (CurrEncValueLeft - CurrEncValueRight)*Sensitivity);
			RightSpeed = speed;
		}
		else if(abs(CurrEncValueLeft) < abs(CurrEncValueRight))
		{
			RightSpeed = (speed - (CurrEncValueRight - CurrEncValueLeft)*Sensitivity);
			LeftSpeed = speed;
		}
		else
		{
			RightSpeed = speed;
			LeftSpeed = speed;
		}

		//If drifting instead of driving straight, don't correct for natural drift
		if(correction != 0)
		{
			RightSpeed = speed;
			LeftSpeed = speed;
		}

		//Set motor speeds
		pwrDriveRight(RightSpeed + correction);
		pwrDriveLeft(LeftSpeed - correction);

		//Update encoder values
		CurrEncValueLeft = getEncLeft();
		CurrEncValueRight = getEncRight();

		if(abs(CurrEncValueRight) > abs(EncValue))
		{  // if the distance is reached
			if(EncValueExceeded &&(EncValueMax != CurrEncValueRight))
			{ //if two consecutive different values over the distance are detected
				exitLoop = true;
			}
			else
			{
				EncValueMax = CurrEncValueRight;
				EncValueExceeded = true;
			}
		} // end if the distance is reached
		else
			EncValueExceeded = false;

		EndTimeSlice();
	} // end while distance not matched

	// Reverse motors to stop coast
	if(driftCorrection){
	    pwrDriveLeft( -speed/2 );
	    pwrDriveRight( -speed/2 );

	    wait1Msec( abs(speed/1.8) );
	}

    // Stop motors
    pwrDrive(0);

} // end EncoderDriveStraight

void autonArc(float distance, int speedL, int speedR, int correction = 5)
{
	long EncValue;                 // # of encoder ticks corresponding to distance
	long CurrEncValueLeft;         //current value of the left encoder
	long CurrEncValueRight;        //current value of the right encoder
	float Sensitivity = 0.15;       //multiplying factor of adjustion
	int RightSpeed;                //calculated speed of right motor
	int LeftSpeed;                 //calculated speed of left motor
	long EncValueMax = 0;          //maximum encoder value recieved
	bool EncValueExceeded = false; //state if the encoder value has been exceeded
	bool exitLoop = false;         //state determining whether the loop should be exited
	int MinSpeed = 50;
	if(MinSpeed>speedL)
		MinSpeed = speedL;

	clearTimer(T4);

	resetEncRight(); //reset encoders
	resetEncLeft();

	wait1Msec(100);

	EncValue = 1120.0 * distance / circumference / 3.5 // /3.0 added

	CurrEncValueLeft = 0;
	CurrEncValueRight = 0;

	while(!exitLoop)
	{ //while distance not matched
		// nxtDisplayTextLine(4, "Right Encoder %.1d", nMotorEncoder[Right1]);
		// nxtDisplayTextLine(5, "Left Encoder %.1d", nMotorEncoder[Left1]);

		//If the wheels are not turning, stop them to prevent motor burnouts
		if(time1[T4] > 300)
		{

			clearTimer(T4);
			if((CurrEncValueRight == LastEncoderValueRight))
			{
				exitLoop = true;
			}
			else
			{
				LastEncoderValueRight = CurrEncValueRight;
			}
		}

		//If drifting instead of driving straight, don't correct for natural drift
		if(correction != 0)
		{
			RightSpeed = speedR;
			LeftSpeed = speedL;
		}

		//Set motor speeds
		pwrDriveRight(RightSpeed + correction);
		pwrDriveLeft(LeftSpeed - correction);

		//Update encoder values
		CurrEncValueRight = getEncRight();

		if(abs(CurrEncValueRight) > abs(EncValue))
		{  // if the distance is reached
			if(EncValueExceeded &&(EncValueMax != CurrEncValueRight))
			{ //if two consecutive different values over the distance are detected
				exitLoop = true;
			}
			else
			{
				EncValueMax = CurrEncValueRight;
				EncValueExceeded = true;
			}
		} // end if the distance is reached
		else
			EncValueExceeded = false;

		EndTimeSlice();
	} // end while distance not matched

    // Stop motors
    pwrDrive(0);

} // end EncoderDriveStraight
#endif


#endif  // _AUTON_LIBRARY
