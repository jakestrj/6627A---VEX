#define Kp 0.05292  // Kp
#define Ki 0.0000000075   // Ki
#define Kd 0.0375   //Kd

task main()
{
  nMaxRegulatedSpeedNxt=500;  // set the max speed as 500
  nMotorEncoder[motorB]=0;  // initialize encoder of motor B
  nMotorEncoder[motorC]=0;  // initialize encoder of motor C
  nMotorPIDSpeedCtrl[motorB] = mtrNoReg;//disable NXT inbuilt PID
  nMotorPIDSpeedCtrl[motorC] = mtrNoReg;//disable NXT inbuilt PID
nSyncedMotors = synchNone;//disable NXT inbuilt PID
  motor[motorB] = 50;//set speed to 500/2=250 degrees per second
  motor[motorC] = 50;//set speed to 500/2=250 degrees per second
  int setPoint=500; // target degrees
  // initialize all variables for both motor B and motor C
  int actualDegreesB=0;
  int actualDegreesC=0;
  int errorB=0;
  int errorC=0;
  int pre_errB=0;
  int pre_errC=0;
  int IB=0;
  int IC=0;
  int DB=0;
  int DC=0;
  float outputB=0;
  float outputC=0;

  time1[T1]=0;
  while(time1[T1]<10000)
  {
    // motor B
    actualDegreesB = nMotorEncoder[motorB];  // find actual degrees of motor B
    errorB=setPoint-actualDegreesB;  // find error of motor B
    IB=IB+errorB;  // find value of integral
    DB=errorB-pre_errB;  // find value of derivative
    outputB=Kp*errorB+Ki*IB+Kd*DB;  // pid contorl for motor B
    pre_errB=errorB;  // store the error
    motor[motorB]=50+outputB*50;

    // motor C
    actualDegreesC = nMotorEncoder[motorC];  // find actual degrees of motor C
    errorC=setPoint-actualDegreesC;  // find error of motor B
    IC=IC+errorC;  // find value of integral
    DC=errorC-pre_errC;  // find value of derivative
    outputC=Kp*errorC+Ki*IC+Kd*DC;  // pid contorl for motor C
    pre_errC=errorC;  // store the error
    motor[motorC]=50+outputC*50;

	// record the value of actual degree of motor C for further research.
	AddToDatalog(1, actualDegreesC);
	// wait for 50 ms
    wait1Msec(50);
  }
  // store all data into a file.
  SaveNxtDatalog();
}
