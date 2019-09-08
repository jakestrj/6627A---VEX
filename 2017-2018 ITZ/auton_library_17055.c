#ifndef  _AUTON_LIBRARY
#define  _AUTON_LIBRARY

#define abs(X) ((X < 0) ? -1 * X : X)
#define neg(X) ((X > 0) ? -1 * X : X)
#define mMGLIFT mgLift
#define mLIFTL liftL
#define mLIFTR liftR
#define mTLIFTY tLiftY
#define mDRIVEL_Y driveLY
#define mDRIVER_Y driveRY
#define mDRIVEL_1 driveL1
#define mDRIVER_1 driveR1
#define mCLAW claw
#define sPOTMG pot_mGoal
#define sPOTLIFT pot_lift
#define sPOTTLIFT pot_tLift
#define sGYRO gyro
#define sENCO encOdo

#define resetEncOdo() SensorValue[sENCO]=0
#define resetGyro() SensorValue[sGYRO]=0
#define resetAll() {resetGyro(); resetEncOdo();}

#define getGyroGlobal() -SensorValue[sGYRO]
#define getPotMG() SensorValue[sPOTMG]
#define getEncLift() SensorValue[sPOTLIFT]
#define getPotTLift() SensorValue[sPOTTLIFT]
#define getEncOdo() SensorValue[sENCO]

#define MOVE_TIMEOUT 1500
#define POT_SENSITIVITY 0.06105006105
#define THRESHOLD_COEFF 2

float encoderCurve (float target) { return 27.2*target; }
float encoderCurveO (float target) {return 35.3*target; }
float reverseEncoderCurveO (float target) {return target/35.3; }
float gyroCurve (float target) { return 10.5*target; }
float reverseGyroCurve(float target) { return target/10.5; }

void pwrDriveRight (int speed){motor[mDRIVER_Y]=motor[mDRIVER_1]=speed;}
void pwrDriveLeft (int speed){motor[mDRIVEL_Y]=motor[mDRIVEL_1]=speed;}
void setLiftMotor(int speed){motor[mLIFTL]=speed; motor[mLIFTR]=speed;}
void setLiftMotor(int left, int right){motor[mLIFTL]=left; motor[mLIFTR]=right;}
void setLiftMotorCross(int speed, int cross){setLiftMotor(speed+cross, speed-cross);}
void setTLiftMotor(int power){motor[mTLIFTY] = power;}
void setMGLiftMotor(int power){motor[mMGLIFT] = power;}
void setClaw(int speed){motor[mCLAW]=speed;}
void spin(int speed){pwrDriveRight(speed); pwrDriveLeft(-speed); }
void arc(int speed1, int speed2){pwrDriveRight(speed1); pwrDriveLeft(speed2); }
void setWheelSpeed(int left, int right){pwrDriveRight(left); pwrDriveLeft(right);}
void setWheelSpeed(int speed){pwrDriveRight(speed); pwrDriveLeft(speed);}
void setWheelSpeedTurn(int speed, int turn){setWheelSpeed(speed+turn, speed-turn);}
int slew (int value, int lastValue,  int slewRate) { if(abs(value-lastValue)>slewRate) { if(lastValue>value) return lastValue-slewRate; else return lastValue+slewRate; } return value; }
int limit (int num, int limit) { return sgn(num)*(abs(num)>limit?limit:abs(num)); }
float trueSpeed(float val){return sgn(val)*(127*pow(fabs(val)/127,1.4));}

enum STATE{remainToBeDone, working, done, pause};
enum POSITION{IN, OUT, IDLE};
bool auto_mgoalin=false,auto_mgoalout=false,auto_mgoalidle=false,auto_idle=false,auto_stack=false,auto_stack_load=false,auto_armin=false,auto_armout=false,auto_clawout=false,auto_clawin=false;
int global_LIFT = 0, global_MGOAL = 0, global_STACKHEIGHT = 0;
bool global_stopMid=false, global_setHigher=false; 


#define AUTO_DEFAULT -999
#define AUTO_MOUT -1
#define AUTO_MIN -2
#define AUTO_MIDLE -3
#define AUTO_IDLE -4
#define AUTO_ARMOUT -5
#define AUTO_ARMIN -6
#define AUTO_CIN -7
#define AUTO_COUT -8
#define AUTO_STACK1 -9
#define AUTO_STACK2 -10
#define _tick_mobilegoalout 755
#define _tick_mobilegoalin 2500
#define _tick_mobilegoalidle 0
#define _tick_idleLimitLower 800
#define _tick_stackbegin 700
#define _tick_stackbeginload 950
#define POT_OFFSET 200 //--------------------
#define _tick_baseHeight 350 //500
#define _tick_tliftout -1
#define _tick_tliftin -1

void putClawIn(void){setClaw(127);delay(500);setClaw(30);}
void setClawOut(void){setClaw(-127);delay(500);setClaw(0);}

void putMobileGoalIn(void){
	setMGLiftMotor(0);
	setMGLiftMotor(-127);
	delay(100);
	do{ delay(50);
		if(motor[mMGLIFT]==0) break;
	}while(getPotMG() < _tick_mobilegoalin);
	setMGLiftMotor(0);
}
void putMobileGoalOut(void){
	setMGLiftMotor(127);
	delay(100);
	do{ delay(50);
		if(motor[mMGLIFT]==0) break;
	}while(getPotMG() > _tick_mobilegoalout);
	setMGLiftMotor(15);
}

/************************************************************************************/
/************************************** STRUCTs **************************************/
/************************************************************************************/
typedef struct {
	float kP, kI, kD, p, i, d;
	int target, error, lastError, integralLimit, threshold, slewRate, lastOutput, lastSensValue, errTot, iActiveZone, unwind, writeCounter; 
	int oldError[50];
	long prevTime;
} pid; 

typedef struct {
	pid odo;
	tSensors odoEncoder, gyroscope;
	int maxSpeed, rangeError, tTarget, startHeading, endHeading;
} drivebase;

typedef struct {
	pid controller;
	tSensors sensor;
	int maxSpeed, rangeError, tTarget;
} gyroscope;

typedef struct {
	pid main;
	tSensors mainEnc;
	int maxSpeed;
} lift;

float updatePIDController (pid *controller, tSensors sensor, bool isAutoLift = false) {
	controller->error = controller->target - (sensor == sGYRO ? -SensorValue[sensor] : SensorValue[sensor]);

  float dT = nPgmTime - controller->prevTime;
	if((sensor == sGYRO || sensor == sPOTLIFT) && !isAutoLift){

		if(dT>1000) dT=0; //no dT cap
	    controller->prevTime = nPgmTime;
	    controller->p = controller->kP*controller->error;
	    controller->d = 0;
	    if(dT!=0) controller->d = (controller->lastError - controller->error) * controller->kD / dT; //reverse sensor if wrong
	    controller->errTot += controller->error * dT;
	    if(abs(controller->error) > controller->iActiveZone) controller->errTot = 0;
	    if(fabs(controller->d) > 10){
	      int maxErrTot = controller->integralLimit / controller->kI;
	      controller->errTot = limit(controller->errTot, maxErrTot);
	    }
	    if((controller->error > 0.0 && controller->errTot < 0.0) || (controller->error < 0.0 && controller->errTot > 0.0) || abs(controller->error) < 0.001){
	    	if((abs(controller->error) - controller->unwind) > -0.001) controller->errTot = 0.0;
	    }
	    if(abs(controller->unwind) < 0.001 && abs(controller->error) < 0.001) controller->errTot = 0;

	  	controller->i = controller->errTot * controller->kI;
			controller->i = (controller->i >= controller->integralLimit) ? 0 : controller->i;

			int output = 0;
			if(sensor == sPOTLIFT){
				output = (controller->i>controller->integralLimit) ? controller->p * 15 : controller->p; //SPECIFIC 15 scaling
				output = slew(output + controller->i + controller->d, controller->lastOutput, controller->slewRate);
			}else output = slew(controller->p + controller->i + controller->d, controller->lastOutput, controller->slewRate);


			controller->oldError[controller->writeCounter] = controller->error*dT; 
			controller->writeCounter %= (sizeof(controller->oldError)/sizeof(controller->oldError[0]))-1; 
			++controller->writeCounter; 
			controller->lastOutput = output;
			controller->lastError = controller->error;
	    	controller->lastSensValue = -SensorValue[sensor];
			return output;

	}else if(sensor == sENCO){
		controller->i += controller->error;
		if(controller->kI*controller->i > controller->integralLimit) controller->i = 10 / controller->kI;
		//controller->i = (controller->i >= controller->integralLimit) ? 0 : controller->i;

		controller->d = (controller->error - controller->lastError);
		controller->lastError = controller->error;

		int output = slew(controller->kP*controller->error + controller->kI*controller->i + controller->kD*controller->d, controller->lastOutput, controller->slewRate);
		controller->lastOutput = output;
		return output;

	}else if(sensor == sPOTLIFT && isAutoLift){
		controller->i += controller->error;
		if(controller->kI*controller->i > controller->integralLimit) controller->i = controller->kI;

		controller->d = (controller->error - controller->lastError);
		controller->lastError = controller->error;

		int output = /*slew(*/controller->kP*controller->error + controller->kI*controller->i + controller->kD*controller->d;/*, controller->lastOutput, controller->slewRate);*/
		controller->lastOutput = output;
		return output;
	}
	return 0;
}

void initPIDController (pid *controller, float kP, float kI, float kD, int unwind, int threshold, int integralLimit, int slewRate) {
	controller->kP = kP;
	controller->kI = kI;
	controller->kD = kD;
	controller->unwind = unwind;
	controller->target = 0;
	controller->error = 0;
	controller->i = 0;
	controller->d = 0;
	controller->lastError = 0;
	controller->threshold = threshold;
	controller->integralLimit = integralLimit;
	controller->slewRate = slewRate;
	controller->lastOutput = 0;
	controller->writeCounter = 0; 
}

void clearIntegral(pid *controller) {
	controller->i = 0;
}

void setTarget(pid *controller, int target) {
	clearIntegral(controller);
	controller->target = target;
}

drivebase db, dbload, dbfast; 
gyroscope g, gload, gfast, gcurve, garc; 
lift l, llock, tlift; 


/************************************************************************************/
/************************************** INITs **************************************/
/************************************************************************************/
void initPIDDrivebase (drivebase *controller, tSensors odoEncoder, float kP,  float kI, float kD, int unwind, int threshold, int integralLimit, int slewRate) {
	initPIDController(controller->odo, kP, kI, kD, unwind, threshold, integralLimit, slewRate);
	controller->odoEncoder = odoEncoder;
}

void initPIDDrivebase (drivebase *controller, tSensors odoEncoder, tSensors gyroscope, float kP,  float kI, float kD, int unwind, int threshold, int integralLimit, int iActiveZone, int slewRate, int rangeError, int tTarget) {
	initPIDDrivebase(controller, odoEncoder, kP,  kI, kD, unwind, threshold, integralLimit, slewRate);
	controller->gyroscope = gyroscope;
	controller->rangeError = rangeError;
	controller->tTarget = tTarget;
}

void initPIDGyroscope (gyroscope *gyroController, tSensors sensor, float kP,  float kI, float kD, int unwind, int threshold, int integralLimit, int iActiveZone, int slewRate, int rangeError, int tTarget) {
	pid *controller = gyroController->controller;
	initPIDController(controller, kP, kI, kD, unwind, threshold, integralLimit, slewRate);
	gyroController->sensor = sensor;
	gyroController->rangeError = rangeError;
	gyroController->tTarget = tTarget;
}

void initPIDLift (lift *controller, tSensors mainEnc, float kP,  float kI, float kD, int unwind, int threshold, int integralLimit, int iActiveZone, int slewRate) {
	initPIDController(controller->main, kP, kI, kD, unwind, threshold, integralLimit, slewRate);
	controller->mainEnc = mainEnc;
}

int getGyroCrossTrackError(int startHeading){
	float kp_gyro=0.15, ki_gyro=0.005; //kp=.15
	int temp_gyro, err_gyro, Intergral_gyro=0, aim_gyro=startHeading; 
	temp_gyro=getGyroGlobal(); 
	err_gyro=aim_gyro-temp_gyro;
	if(abs(err_gyro)<3) err_gyro=0;
	if(abs(err_gyro)<15) Intergral_gyro+=err_gyro;
	return (kp_gyro*err_gyro+ki_gyro*Intergral_gyro);
}


/************************************************************************************/
/************************************** PIDs **************************************/
/************************************************************************************/
task MobileGoalControl(){
	enum STATE stata = done;
	int aim = AUTO_DEFAULT;
	//lift *controller = ml;
	while(true){
		if(auto_mgoalin){
			auto_mgoalin = false;
			aim = AUTO_MIN;
			stata = remainToBeDone;
		}
		if(auto_mgoalout){
			auto_mgoalout = false;
			aim = AUTO_MOUT;
			stata = remainToBeDone;
		}else if(auto_mgoalidle){
			auto_mgoalout = false;
			aim = AUTO_MIDLE;
			stata = remainToBeDone;
		}
		if(aim == AUTO_MIN && stata != done){
			putMobileGoalIn();
			stata = done;
		}else if(aim == AUTO_MOUT && stata != done){
			putMobileGoalOut();
			stata = done;
		}
		else if(aim == AUTO_MIDLE && stata != done){
			/*controller->maxSpeed = 127;
			setTarget(controller->main, _tick_mobilegoalidle);
			liftPID(controller); */
			stata = done;
		}
		delay(20);
	}
}
void autonMG(enum POSITION target){
	stopTask(MobileGoalControl);
	startTask(MobileGoalControl);
	if(target==OUT) auto_mgoalout=true;
	else if(target==IN) auto_mgoalin=true;
	else if(target==IDLE) auto_mgoalidle=true;
}

bool drivePID(drivebase *controller, int heading=getGyroGlobal(), bool mgoal=false)
{
	resetEncOdo();
	pid *odo = controller->odo;
	int lastTime = nPgmTime;
	short timeAtTarget = 0;

	writeDebugStream("INTENDED: (O: %d) (G: %d) ", odo->target, heading);

	controller->startHeading = heading; 
	gyroscope *gyroControllerCurve = gcurve;
	pid *gyroPidCurve = gyroControllerCurve->controller;
	setTarget(gyroPidCurve, controller->startHeading);

	clearTimer(T2);	clearTimer(T3);

	bool mgoalstarted = false, recordError = false; 
	int startError = 0; 

	do{
	    if (time1(T3) >= MOVE_TIMEOUT) { setWheelSpeed(0); writeDebugStreamLine("\nENDED FALSE"); controller->endHeading = getGyroGlobal(); return false; }

	    lastTime = nPgmTime;

		setWheelSpeedTurn(
				limit(updatePIDController(odo, controller->odoEncoder), controller->maxSpeed),
				limit(updatePIDController(gyroPidCurve, gyroControllerCurve->sensor), controller->maxSpeed)
				//getGyroCrossTrackError(controller->startHeading)
		);
		if(!recordError){ startError = odo->error; recordError=true; }

		//writeDebugStreamLine("Err: %d Sens: %d", odo->error, SensorValue[controller->odoEncoder]);
		delay(10);

		if(!mgoalstarted && mgoal && abs(odo->error)<abs(0.85*startError)){
			writeDebugStreamLine("STARTED MGOAL"); 
			mgoalstarted=true; 
			autonMG(OUT); 
		}

		if(odo->error <= controller->rangeError && odo->error >= -controller->rangeError) {
			timeAtTarget = time1(T2);
			if(timeAtTarget >= controller->tTarget) {
				setWheelSpeed(0);
				controller->endHeading = getGyroGlobal();
				writeDebugStreamLine("\nENDED TRUE");
				return true;
			}
		}
		else clearTimer(T2); 

		delay(10);

	} while(true);

	//writeDebugStreamLine("ACT: (%d)", SensorValue[controller->odoEncoder]);
	return true;
}

bool arcTurnGyroPID (gyroscope *gyroController, int offsetSpeed) {
	resetAll();
	pid *controller = gyroController->controller;
	short timeAtTarget = 0;
	writeDebugStream("INTENDED: (%d) ", controller->target);
	clearTimer(T3); 
	clearTimer(T2);

	do {
		//if (time1(T3) >= MOVE_TIMEOUT) { setWheelSpeed(0); writeDebugStreamLine("\nENDED FALSE"); return false; }

		arc(limit(updatePIDController(controller, gyroController->sensor), gyroController->maxSpeed), 
			limit(updatePIDController(controller, gyroController->sensor), offsetSpeed));

		//writeDebugStreamLine("Err: %d Sens: %d", controller->error, getGyroGlobal());

		delay(10);
		if(controller->error <= gyroController->rangeError && controller->error >= -gyroController->rangeError) {
			timeAtTarget = time1(T2);
			if(timeAtTarget >= gyroController->tTarget) {
				if(gyroController->tTarget==0){
					setWheelSpeed(gyroController->maxSpeed/2*-sgn(controller->target), offsetSpeed/2 *-sgn(controller->target) );
					delay(gyroController->maxSpeed/1.5);
				}
				setWheelSpeed(0);
        		writeDebugStreamLine("\nENDED TRUE");
				return true;
			}
		}
		else clearTimer(T2);

		delay(10);

	} while(true);

	//writeDebugStreamLine("ACT: (%d)", SensorValue[gyroController->sensor]);
	return true;
}

bool pointTurnGyroPID (gyroscope *gyroController) {
	resetAll();
	pid *controller = gyroController->controller;
	short timeAtTarget = 0;
	writeDebugStream("INTENDED: (%d) ", controller->target);
	clearTimer(T3); 
	clearTimer(T2);

	do {
		//if (time1(T3) >= MOVE_TIMEOUT) { setWheelSpeed(0); writeDebugStreamLine("\nENDED FALSE"); return false; }

		spin(limit(updatePIDController(controller, gyroController->sensor), gyroController->maxSpeed));

		//writeDebugStreamLine("Err: %d Sens: %d", controller->error, getGyroGlobal());

		delay(10);
		if(controller->error <= gyroController->rangeError && controller->error >= -gyroController->rangeError) {
			timeAtTarget = time1(T2);
			if(timeAtTarget >= gyroController->tTarget) {
				if(gyroController->tTarget==0){
					spin(gyroController->maxSpeed/2 * -sgn(controller->target));
					delay(gyroController->maxSpeed/1.5);
				}
				setWheelSpeed(0);
        		writeDebugStreamLine("\nENDED TRUE");
				return true;
			}
		}
		else clearTimer(T2);

		delay(10);

	} while(true);

	//writeDebugStreamLine("ACT: (%d)", SensorValue[gyroController->sensor]);
	return true;
}

/* initPIDDrivebase (drivebase *controller, tSensors odoEncoder, tSensors gyroscope, float kP,  float kI, float kD, int unwind,
int threshold, int integralLimit, int iActiveZone, int slewRate, int rangeError, int tTarget)

 initPIDGyroscope (gyroscope *gyroController, tSensors sensor, float kP,  float kI, float kD, int unwind,
 int threshold, int integralLimit, int iActiveZone, int slewRate, int rangeError, int tTarget)

 void initPIDLift (lift *controller, tSensors mainEnc, float kP,  float kI, float kD, int unwind, 
 int threshold, int integralLimit, int iActiveZone, int slewRate)
*/
/************************************************************************************/
/************************************** FNXs ****************************************/
/************************************************************************************/
void initPids(void){
	initPIDDrivebase(db, sENCO, sGYRO,     0.285, 0, 1.5, 0, 100, 30, 40, 50, 90, 250);
	initPIDDrivebase(dbload, sENCO, sGYRO, 0.15, 0, 1.5, 0, 100, 30, 40, 50, 90, 225);
	initPIDGyroscope(g, sGYRO,             0.25, 0.002, 0, 15, 100, 40, 40, 20, 90, 250); //.35
	initPIDGyroscope(gload, sGYRO,         0.35, 0.002, 15, 0, 100, 40, 40, 20, 90, 250);
	initPIDGyroscope(gfast, sGYRO,         0.5, 0.001, 0.001, 0, 100, 40, 40, 127, 90, 250);
	initPIDGyroscope(gcurve, sGYRO,        0.5, 0.002, 15, 0, 100, 40, 40, 20, 90, 0);
	initPIDGyroscope(garc, sGYRO,          0.5, 0.002, 15, 0, 100, 30, 40, 50, 90, 0); //250
	initPIDLift     (l, sPOTLIFT,          0.05, 0, 0, 1, 110, 25, 10, 30); //0.12
	initPIDLift     (tlift, sPOTTLIFT,     0.05, 0, 0, 1, 110, 25, 10, 30); //0.12
	initPIDLift     (llock, sPOTLIFT,      0.25, 0, 0, 1, 150, 25, 10, 30); //p=.25
}

void autonDrive(drivebase *controller, float target, int maxSpeed, int wait, int heading=-1) {
	controller->maxSpeed = maxSpeed;
	setTarget(controller->odo, encoderCurveO(target));
	if(heading==-1) drivePID(controller);
	else drivePID(controller, heading);
	delay(wait);
}
void autonDriveM(drivebase *controller, float target, int maxSpeed, int wait, int heading=-1) {
	controller->maxSpeed = maxSpeed;
	setTarget(controller->odo, encoderCurveO(target));
	if(heading==-1) drivePID(controller, getGyroGlobal(), true);
	else drivePID(controller, heading, true);
	delay(wait);
}
void autonTurn (gyroscope *gyroController, float target, int maxSpeed, int wait) {
	gyroController->maxSpeed = maxSpeed;
	setTarget(gyroController->controller, gyroCurve(target));
	pointTurnGyroPID(gyroController);
	delay(wait);
}
void autonArc (gyroscope *gyroController, float target, int speedHigh, int speedOffset, int wait) {
	gyroController->maxSpeed = speedHigh;
	setTarget(gyroController->controller, gyroCurve(target));
	arcTurnGyroPID(gyroController, speedOffset);
	delay(wait);
}

void autonDrive(float target, int maxSpeed, int wait, int heading=-1){ autonDrive(db, target, maxSpeed, wait, heading); }
void autonDriveM(float target, int maxSpeed, int wait){ autonDriveM(db, target, maxSpeed, wait); }
void autonDriveLoad(float target, int maxSpeed, int wait){ autonDrive(dbload, target, maxSpeed, wait); }
void autonDriveFast(float target, int maxSpeed, int wait){ autonDrive(dbfast, target, maxSpeed, wait); }
void autonTurn(float target, int maxSpeed, int wait){ autonTurn(g, target, maxSpeed, wait); }
void autonTurnLoad(float target, int maxSpeed, int wait){ autonTurn(gload, target, maxSpeed, wait); }
void autonTurnFast(float target, int maxSpeed, int wait){ autonTurn(gfast, target, maxSpeed, wait); }
void autonArc(float target, int speedHigh, int speedOffset, int wait){autonArc(garc, target, speedHigh, speedOffset, wait);}
float _dist, _del; 
task autonDriveT(){autonDrive(_dist, 127, _del);}
void autonDriveTask(int dist, int speed, int del){_dist=dist;_del=del;stopTask(autonDriveT);startTask(autonDriveT);}
task autonTurnT(){autonTurn(_dist, 127, _del);}
void autonTurnTask(int dist, int speed, int del){_dist=dist;_del=del;stopTask(autonTurnT);startTask(autonTurnT);}

/************************************************************************************/
/************************************** LIFTs **************************************/
/************************************************************************************/
task TLiftControl(){
	enum STATE stata = done;
	int aim = AUTO_DEFAULT;

	while(true){
		if(auto_armout){
			auto_armout=false;
			aim = AUTO_ARMOUT;
			stata = remainToBeDone;
		}else if(auto_armin){
			auto_armin = false;
			aim = AUTO_ARMIN;
			stata=remainToBeDone;
		}
		if(aim == AUTO_ARMOUT && stata != done){
			setLiftMotor(0); 
			setTLiftMotor(127);
			delay(300);
			setTLiftMotor(11);
			stata = done;
		}
		if(aim == AUTO_ARMIN && stata != done){
			setLiftMotor(0); 
			setTLiftMotor(-127);
			delay(400);
			setTLiftMotor(-12);
			stata = done;
		}
		delay(20);
	}
}

task ClawControl(){
	enum STATE stata = done;
	int aim = AUTO_DEFAULT;
	while(true){
		if(auto_clawout){
			auto_clawout = false;
			aim = AUTO_COUT;
			stata = remainToBeDone;
		}
		if(auto_clawin){
			auto_clawin = false;
			aim = AUTO_CIN;
			stata = remainToBeDone;
		}
		if(aim == AUTO_COUT && stata != done){
			setClawOut();
			stata = done;
		}
		if(aim == AUTO_CIN && stata != done){
			putClawIn();
			stata = done;
		}
		delay(20);
	}
}

void autonClaw(enum POSITION target){
	stopTask(ClawControl);
	startTask(ClawControl);
	if(target==OUT) auto_clawout=true;
	else if(target==IN) auto_clawin=true;
}

void autonArm(enum POSITION target){
	stopTask(TLiftControl);
	startTask(TLiftControl);
	if(target==OUT) auto_armout=true;
	else if(target==IN) auto_armin=true;
}

bool liftPID(lift *controller, bool stacker = false, bool up = false, bool driver=false)
{
	pid *lift = controller->main;
	writeDebugStream("INTENDED: (L: %d) ", lift->target);
	bool close = false;

	clearTimer(T2); clearTimer(T3);

	do{
    	setLiftMotor(
				limit(updatePIDController(lift, controller->mainEnc, stacker), controller->maxSpeed),
		);

		//writeDebugStreamLine("Err: %d Sens: %d Stop: %d", lift->error, getEncLift(), stacker);
		delay(10);

		if(!close && up && abs(lift->error) < 250){ autonArm(IN); close=true;}

		if(stacker && lift->error <= lift->threshold && lift->error >= -lift->threshold ) {
			writeDebugStreamLine("\nENDED TRUE");
			break;
		}

		//writeDebugStreamLine("Err: %d Sens: %d dT: %d", lift->error, SensorValue[controller->mainEnc]);

		delay(25);

	} while(true);

	writeDebugStreamLine("ACT: (%d)", SensorValue[controller->mainEnc]);
	return true;
}

task LiftControl(){
	enum STATE stata = done;
	int aim = AUTO_DEFAULT;
	lift *controller = l; //l
	while(true){
		if(auto_idle){
			auto_idle = false;
			aim = AUTO_IDLE;
			stata = remainToBeDone;
		}
		if(aim == AUTO_IDLE && stata != done){
			controller->maxSpeed = 127;
			global_LIFT-=POT_OFFSET; 
			setTarget(controller->main, global_LIFT);
			liftPID(controller);
			stata = done;
		}
		delay(20);
	}
}

void autonLift(float target){
	stopTask(LiftControl);
	startTask(LiftControl);
	global_LIFT=target;
	auto_idle=true;
}

bool sreset = false; //resetinop
enum STATE stata_autostacker = done;
task AutoStacker(){
	int aim = AUTO_DEFAULT;
	lift *controller = llock; //use only this config
	while(true){
		if(auto_stack){
			auto_stack = false;
			aim = AUTO_STACK1;
			stata_autostacker = remainToBeDone;
		}
		if(aim == AUTO_STACK1 && stata_autostacker != done){
			stopTask(LiftControl);
			int aim = limit(global_STACKHEIGHT*(global_STACKHEIGHT<2 ? 80 : 110) + _tick_stackbegin, 2850);
			controller->maxSpeed = 127;
			setTarget(controller->main, aim);
			liftPID(controller, true, true);
			delay(100);
			setTarget(controller->main, aim-300);
			liftPID(controller, true, false);
			delay(300);
			autonClaw(OUT);
			if(!global_setHigher){
				setTarget(controller->main, getEncLift()+100);
				liftPID(controller, true, false);
			}
			if(!global_stopMid){	
				delay(100);
				autonArm(OUT);
				delay(550);
				controller->maxSpeed = 120;
				setTarget(controller->main, _tick_baseHeight);
				liftPID(controller, true, false);
			}else if(global_stopMid && global_setHigher){
				setTarget(controller->main, 1200);
				liftPID(controller, false, false);
			}
			++global_STACKHEIGHT;
			sreset = false;
			setClaw(127);
			stata_autostacker = done;
			global_stopMid=false; 
		}
		delay(20);
	}
}

enum STATE stata_autostackerload = done;
task AutoStackerLoad(){
	int aim = AUTO_DEFAULT;
	lift *controller = llock; //use only this config
	while(true){
		if(auto_stack_load){
			auto_stack_load = false;
			aim = AUTO_STACK2;
			stata_autostackerload = remainToBeDone;
		}
		if(aim == AUTO_STACK2 && stata_autostackerload != done){
			stopTask(LiftControl);
			int aim = limit(global_STACKHEIGHT*(global_STACKHEIGHT<4 ? 100 : 125) + _tick_stackbegin, 2850);
			controller->maxSpeed = 127;
			writeDebugStreamLine("------------ AIM: %d STACK: %d", aim, global_STACKHEIGHT);
			if(global_STACKHEIGHT==1){
				stopTask(LiftControl); 
				setClaw(127); 
				setTarget(controller->main, _tick_stackbeginload-200);
				liftPID(controller, true, false);
				delay(150); 
			}
			setTarget(controller->main, (global_STACKHEIGHT<5 ? _tick_stackbeginload+100 : getEncLift()+150));
			liftPID(controller, true, true); //tbar in 			
			delay(200);
			setTarget(controller->main, aim-200);
			liftPID(controller, true, false);
			delay(100);
			autonClaw(OUT);
			delay(50); 
			setTarget(controller->main, (global_STACKHEIGHT<5 ? aim+100 : getEncLift()+150));
			liftPID(controller, true, false);
			if(!global_stopMid){
				delay(100);
				autonArm(OUT);
				delay(550);
				if(!global_stopMid) setClaw(127);
				controller->maxSpeed = 127;
				setTarget(controller->main, _tick_stackbeginload-200);
				liftPID(controller, true, false);
			}
			++global_STACKHEIGHT;
			sreset = false;
			stata_autostackerload = done;
			global_stopMid=false; 
		}
		delay(20);
	}
}

void waitForAutonStack(){
	do{delay(50);
	}while(stata_autostacker!=done);
}

void waitForAutonStackLoad(){
	do{delay(50);
	}while(stata_autostackerload!=done);
}

void autonStack(bool stopMid=false, bool autonSetHigher=false){
	stopTask(LiftControl);
	setLiftMotor(0);
	global_stopMid = stopMid;
	global_setHigher = autonSetHigher; 
	stopTask(AutoStackerLoad); 
	stopTask(AutoStacker);
	startTask(AutoStacker);
	auto_stack=true;
}

void autonStackLoad(bool stopMid=false){
	stopTask(LiftControl);
	setLiftMotor(0);
	global_stopMid = stopMid;
	stopTask(AutoStacker); 
	stopTask(AutoStackerLoad);
	startTask(AutoStackerLoad);
	auto_stack_load=true;
}

#endif  // _AUTON_LIBRARY
