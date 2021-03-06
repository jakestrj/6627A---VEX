#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           DLOne,         tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           DLTwo,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           FLOne,         tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port4,           FLTwo,         tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           Intake,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           FRTwo,         tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           FROne,         tmotorVex393TurboSpeed_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port9,           DRTwo,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          DROne,         tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


long GetVelocity ()
{
	return( nMotorEncoder[ FROne ] );
}


task shooterControl () {
	float kp = 0;
	float ki = 0;
	float kd = 0;



	float current = 0;
	float circ = 5 * 3.141592653589;

	float IntegralActiveZone = ((1 * 12 / circ ) * 360 );
	float errorT;
	float lastError;
	float proportion;
	float integral;
	float derivative;
	float power;
	float velocity;

	while( true ) {

		velocity = GetVelocity();

		float error = ((power*12/circ)*261.333) - velocity;

		if(error < IntegralActiveZone && error != 0)
		{
			errorT += error;
		}
		else{
			errorT = 0;
		}
		if(errorT > 50/ki)
		{
			errorT = 50/ki;
		}
		if(error == 0)
		{
			derivative = 0;
		}
		proportion	=	error		*		kp;
		integral		=	errorT	*		ki;
		derivative	=	(error	-	lastError)	*	kd;

		lastError	= error;



		current = proportion + integral + derivative;

		if(power == 0) {
			current = 0;
		}

		if(current < 0) {
			current = 0;
		}
		motor[ FROne ] = motor[ FRTwo ] = motor[ FLOne ] = motor[ FLTwo ] = current;

		if(vexRT[Btn6D] == 1) {
			motor[ FROne ] = motor[ FRTwo ] = motor[ FLOne ] = motor[ FLTwo ] = 0;
		}
		wait1Msec(20);
	}
}



task main()
{
	startTask( shooterControl );

	int power;

	while(1)
	{
		if( vexRT[ Btn8L ] == 1 )
			power = 100;
		if( vexRT[ Btn5U ] == 1 )
			motor[ Intake ] = 70;
		if( vexRT[ Btn5D ] == 1 )
			motor[ Intake ] = -70;
		//Right side of the robot is controlled by the right joystick, Y-axis
		motor[ DROne ] = vexRT[Ch2];
		motor[ DRTwo ] = vexRT[Ch2];
		//Left side of the robot is controlled by the left joystick, Y-axis
		motor[ DLOne ] = vexRT[Ch3];
		motor[ DLTwo ] = vexRT[Ch3];

		// Don't hog cpu
		wait1Msec(10);
	}

}
