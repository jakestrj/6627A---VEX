#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
#pragma config(Sensor, in2,    gyro,           sensorGyro)
#pragma config(Sensor, in3,    pot_tLift,      sensorPotentiometer)
#pragma config(Sensor, in4,    pot_mGoal,      sensorPotentiometer)
#pragma config(Sensor, in5,    pot_lift,       sensorPotentiometer)
#pragma config(Sensor, in8,    gyro,           sensorNone)
#pragma config(Sensor, dgtl1,  encOdo,         sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  encLiftL,       sensorQuadEncoder)
#pragma config(Motor,  port1,           mgLift,        tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           driveL1,       tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           driveLY,       tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port4,           tLiftY,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           liftL,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           liftR,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           claw,          tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           driveRY,       tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           driveR1,       tmotorVex393HighSpeed_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#pragma competitionControl(Competition)
static int autonomousMode = 0;
#include "Vex_Competition_Includes_M.c"
#include "auton_library_17055.c"
#include "getlcdbuttons.c"

/************************************************************************************/
/************************************* Setting **************************************/
/************************************************************************************/
#define mainJoyY vexRT[Ch4] //ch2
#define mainJoyX vexRT[Ch2] //ch4
#define secondaryJoyY Ch4
#define secondaryJoyX Ch2

/************************************************************************************/
/******************************** Automous ******************************************/
/************************************************************************************/
#define MAX_CHOICE  4
#define MAX_CHOICE_LCD  3
static int lcdTelemMode = 0;
bool mAutonBool = false;
int autonNumber = 0;

void pre_auton(){bStopTasksBetweenModes = true;}

task writeC(){while(1){writeDebugStreamLine("TIME:%d", time1[T1]); delay(100);}}

#include "autons_raw_17055.c"
task autonomous(){
	resetAll();
	initPids();
	clearDebugStream();
	clearLCDLine(0);clearLCDLine(1);
	global_STACKHEIGHT=0;
	clearTimer(T1);
	//startTask(writeC);
	switch (autonomousMode){
	case 0:
		auton1_2(true);
		//auton1_3(true);
		//auton1_4(true);
		//autonLoad(false);
		//autonLoad5();
		break;
	case 1:
		autonStat();
		break;
	case 2:
		auton1_3_in5(true);
		break;
	case 3:
		auton1_3_in5(false);
		break;
	case 4:
		break;
	default:
		break;
	}
}
/************************************************************************************/
/******************************* User control ***************************************/
/************************************************************************************/
void LcdAutonomousSet( int value, bool select = false )
{
	if(select) autonomousMode = value;
	if(autonomousMode == value) mAutonBool = true;
	else mAutonBool = false;
}

task LcdAutonomousSelection()
{
	TControllerButtons  button; autonNumber = 0; bLCDBacklight = true; mAutonBool = false; LcdAutonomousSet(0);
	while( true ){
		button = getLcdButtons();
		if( ( button == kButtonLeft ) || ( button == kButtonRight ) ) {
			if( button == kButtonLeft )
				if( --autonNumber < 0 ) autonNumber = MAX_CHOICE;
			if( button == kButtonRight){
				++lcdTelemMode;
				if( lcdTelemMode > MAX_CHOICE_LCD ) lcdTelemMode = 0;
				clearLCDLine(0);
			}
			LcdAutonomousSet(autonNumber);
		}
		if( button == kButtonCenter)
			LcdAutonomousSet( autonNumber, true );
		delay(25);
	}
}

void FwLcdControl_user()
{
	clearLCDLine(0);
	char str0[32]; char str1[32];
	switch(lcdTelemMode){
	case 0:
		sprintf( str0, "G:%d M:%d", getGyroGlobal(), getPotMG());
		break;
	case 1:
		sprintf( str0, "O:%d T:%d", getEncOdo(), getPotTLift());
		break;
	case 2:
		sprintf( str0, "L:%d", getEncLift());
		break;
	default:
		clearLCDLine(0); break;
	}
	sprintf( str1, "B(%d)%d %1.2f", mAutonBool, autonNumber, nImmediateBatteryLevel/1000.0 );
	displayLCDString(0, 0, str0);
	displayLCDString(1, 1, str1);
}

task usercontrol()
{
	resetAll();
	initPids();
	clearDebugStream();
	stopTask(writeC);
	stopTask(MobileGoalControl);
	stopTask(LiftControl);
	stopTask(TLiftControl);
	stopTask(MobileGoalControl);
	stopTask(ClawControl);

	startTask(LcdAutonomousSelection);

	int threshold = 20;
	int mainX = 0,mainY = 0,secondaryX = 0,secondaryY = 0;
	bool reset = true, reset_t = false, reset_t_back = false, forceRestart=false;
	bool cl=false;

	while(1)
	{
		FwLcdControl_user();
		mainX=trueSpeed(mainX); mainY=trueSpeed(mainY);
		if(abs(mainJoyY) >= threshold)	mainY = /*log(256/(mainJoyY+128)-1)/(-.025)*/mainJoyY;
		else	mainY = 0;
		if(abs(mainJoyX) >= threshold)	mainX = /*log(256/(mainJoyX+128)-1)/(-.025)*/mainJoyX;
		else	mainX = 0;
		if(abs(mainY) < abs(secondaryY)) mainY = secondaryY;
		if(abs(mainX) < 40) mainX /= 2;

		pwrDriveRight(-(-mainY - -mainX));
		pwrDriveLeft(-mainX + -mainY);

		/*if(vexRT[Btn8U]){if(!forceRestart && !sreset && getEncLift()<1900){ autonLift(1900);} setMGLiftMotor(127); }
		else if(vexRT[Btn8D]){if(!forceRestart && !sreset && getEncLift()<1900){ autonLift(1900);} setMGLiftMotor(-127); }
		else {
			setMGLiftMotor(0);
		}*/

		if(vexRT[Btn8U]) setMGLiftMotor(127);
		else if(vexRT[Btn8D]) setMGLiftMotor(-127);
		else { if(!sreset && getPotMG()<_tick_mobilegoalout) setMGLiftMotor(15); else setMGLiftMotor(0); }

		if(vexRT[Btn5U]){ setClaw(127); cl=true; }
		else if(vexRT[Btn5D]){ setClaw(-127); cl=false; }
		else{
			if(cl && !sreset) setClaw(30);
			else if(!sreset) setClaw(0);
		}

		if(vexRT[Btn7L]) forceRestart=true;

		//if(vexRT[Btn7U]) setTLiftMotor(127);
		//else if(vexRT[Btn7D]) setTLiftMotor(-127);
		//else setTLiftMotor(0);

		if(vexRT[Btn7U]){reset_t=true; reset_t_back=false; setTLiftMotor(127); }
		else if(vexRT[Btn7D]){ reset_t=false; reset_t_back=true; setTLiftMotor(-127); }
		else if(!sreset){ if(reset_t && !forceRestart && (vexRT[Btn6U]>0 || vexRT[Btn6D]>0)) setTLiftMotor(7);
			else if(reset_t_back && !forceRestart) setTLiftMotor(-11); else setTLiftMotor(0); }

		if(vexRT[Btn6U]){ stopTask(LiftControl); setLiftMotor(127);
			reset=false; forceRestart=false; sreset=false; }
		else if(vexRT[Btn6D]){ stopTask(LiftControl); setLiftMotor(-127);
			reset=false; forceRestart=false; sreset=false; }
		else{
			if(forceRestart){ stopTask(LiftControl); setLiftMotor(0); }
			if(getEncLift()<_tick_idleLimitLower && !forceRestart && !sreset) {stopTask(LiftControl);setLiftMotor(-20);}
			else if(!reset && !forceRestart && !sreset){
				autonLift(getEncLift()); reset=true;
			}
		}

		delay(30);
	}
}