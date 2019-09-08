// Wrap code with definition
#ifndef  _AUTON_LIBRARY
#define  _AUTON_LIBRARY

#define abs(X) ((X < 0) ? -1 * X : X)
#define neg(X) ((X > 0) ? -1 * X : X)
#define mMGLIFTR mgLiftR
#define mMGLIFTL mgLiftL
#define mDRIVER_Y1 driveLY1
#define mDRIVER_Y2 driveLY2
#define mDRIVEL_Y1 driveRY1
#define mDRIVEL_Y2 driveRY2
#define mCLAW claw
#define mARM arm
#define sPOTMGOAL pot_mGoal
#define sGYRO gyro
#define sENCO encOdo

void resetGyro(){SensorValue[sGYRO]=0;}
#define resetEncOdo() SensorValue[sENCO]=0
#define resetAll() {resetGyro(); resetEncOdo();}

#define getSensorPotMGoal() (float) abs(SensorValue[sPOTMGOAL])
#define getGyroGlobal() -SensorValue[sGYRO]

#define MOVE_TIMEOUT 1200
#define THRESHOLD_COEFF 2

float encoderCurve (float target) { return 27.2*target; }
float encoderCurveO (float target) {return 35.3*target; }
float gyroCurve (float target) { return 10.5*target; }
float reverseGyroCurve(float target) { return target/-10.5; }

void pwrClaw(int speed){motor[mCLAW]=speed;}
void pwrArm(int speed){motor[mARM]=speed;}
void setMGLiftMotor(int power){motor[mMGLIFTR] = motor[mMGLIFTL] = power;}
void pwrDriveRight (int speed){motor[mDRIVER_Y1]=motor[mDRIVER_Y2]=speed;}
void pwrDriveLeft (int speed){motor[mDRIVEL_Y1]=motor[mDRIVEL_Y2]=-speed;}
void spin(int speed){pwrDriveRight(speed); pwrDriveLeft(-speed); }
void setWheelSpeed(int left, int right){pwrDriveRight(left); pwrDriveLeft(right);}
void setWheelSpeed(int speed){pwrDriveRight(speed); pwrDriveLeft(speed);}
void setWheelSpeedTurn(int speed, int turn){setWheelSpeed(speed+turn, speed-turn);}
int slew (int value, int lastValue,  int slewRate) { if(abs(value-lastValue)>slewRate) { if(lastValue>value) return lastValue-slewRate; else return lastValue+slewRate; } return value; }
int limit (int num, int limit) { return sgn(num)*(abs(num)>limit?limit:abs(num)); }

/************************************************************************************/
/************************************** STRUCTs **************************************/
/************************************************************************************/
typedef struct {
	float kP;
	float kI;
	float kD;
	int target;
	int error;
	int integral;
	int derivative;
	int lastError;
	int integralLimit;
	int threshold;
	int slewRate;
	int lastOutput;
  	int lastSensValue; 
  	int errTot; 
  	int iActiveZone; 
  	int unwind; 
	long prevTime; 
} pid;

typedef struct {
	pid left;
	pid right;
	pid odo;
	tSensors odoEncoder;
	tSensors gyroscope;
	int maxSpeed;
	int rangeError;
	int tTarget;
	int startHeading;
	int endHeading;
} drivebase;

typedef struct {
	pid controller;
	tSensors sensor;
	int maxSpeed;
	int rangeError;
	int tTarget;
} gyroscope;

float updatePIDController (pid *controller, tSensors sensor) {
	if(sensor == sGYRO){
		controller->error = controller->target - getGyroGlobal(); 
	}else controller->error = controller->target - (-SensorValue[sensor]);

  	float p,i,d; 
  	unsigned long dT = nPgmTime - controller->prevTime;
	if(sensor == sGYRO){
	    if(dT>1000) dT=0; 
	    controller->prevTime = nPgmTime; 
	    p = controller->kP*controller->error; 
	    d = 0; 
	    if(dT!=0) d = (controller->lastSensValue - (-SensorValue[sensor])) * controller->kD / dT; 
	    controller->errTot += controller->error * dT; 
	    if(abs(controller->error) > controller->iActiveZone) controller->errTot = 0; 
	    if(abs(d) > 10){
	      int maxErrTot = controller->integralLimit / controller->kI;
	      controller->errTot = limit(controller->errTot, maxErrTot);
	    }
	    if(abs(controller->error) < 0.001) controller->errTot = 0; 
	    i = controller->errTot * controller->kI; 

		controller->integral += controller->error;
		controller->integral = (controller->integral >= controller->integralLimit && controller->integralLimit != -1) ? 0 : controller->integral;
		controller->derivative = (controller->error - controller->lastError) ; // /25 turn, /10 drive
		controller->lastError = controller->error;

		int output = slew(p + i + d, controller->lastOutput, controller->slewRate);
		controller->lastOutput = output;
    	controller->lastSensValue = -SensorValue[sensor]; 
		return output;

	}else if(sensor == sENCO){
	    /*if(dT>1000) dT=0; 
	    controller->prevTime = nPgmTime; 
	    p = controller->kP*controller->error; 
	    d = 0; 
	    if(dT!=0) d = (controller->lastSensValue - (-SensorValue[sensor])) * controller->kD / dT; 
	    controller->errTot += controller->error * dT; 
	    if(abs(controller->error) > controller->iActiveZone) controller->errTot = 0; 
	    if(abs(d) > 10){
	      int maxErrTot = controller->integralLimit / controller->kI; 
	      controller->errTot = limit(controller->errTot, maxErrTot); 
	    }
	    if(abs(controller->error) < 0.001) controller->errTot = 0; 
	    i = controller->errTot * controller->kI; */
		controller->integral += controller->error;
		if(controller->kI*controller->integral > 10) controller->integral = 10 / controller->kI;
		controller->integral = (controller->integral >= controller->integralLimit && controller->integralLimit != -1) ? 0 : controller->integral;

		controller->derivative = (controller->error - controller->lastError);
		controller->lastError = controller->error;

		int output = slew(controller->kP*controller->error + controller->kI*controller->integral + controller->kD*controller->derivative, controller->lastOutput, controller->slewRate);
		controller->lastOutput = output;
		return output;
	}
	return 0;
}

void initPIDController (pid *controller, float kP, float kI, float kD, int threshold = 10, int integralLimit = -1, int slewRate = 127) {
	controller->kP = kP;
	controller->kI = kI;
	controller->kD = kD;
	controller->target = 0;
	controller->error = 0;
	controller->integral = 0;
	controller->derivative = 0;
	controller->lastError = 0;
	controller->threshold = threshold;
	controller->integralLimit = integralLimit;
	controller->slewRate = slewRate;
	controller->lastOutput = 0;
}

void clearIntegral(pid *controller) {
	controller->integral = 0;
}

void setTarget(pid *controller, int target) {
	clearIntegral(controller);
	controller->target = target;
}

drivebase db, dbload, dbfast;
gyroscope g, gload, gfast, gcurve;

/************************************************************************************/
/************************************** INITs **************************************/
/************************************************************************************/

void initPIDDrivebase (drivebase *controller, tSensors odoEncoder, float kP,  float kI, float kD, int threshold = 10, int integralLimit = -1, int slewRate = 10) {
	initPIDController(controller->odo, kP, kI, kD, threshold,  integralLimit, slewRate);
	controller->odoEncoder = odoEncoder;
}

void initPIDDrivebase (drivebase *controller, tSensors odoEncoder, tSensors gyroscope, float kP,  float kI, float kD, int threshold, int integralLimit, int iActiveZone, int slewRate, int rangeError, int tTarget) {
	initPIDDrivebase(controller, odoEncoder, kP,  kI, kD, threshold, integralLimit, slewRate);
	controller->gyroscope = gyroscope;
	controller->rangeError = rangeError;
	controller->tTarget = tTarget;
}

void initPIDGyroscope (gyroscope *gyroController, tSensors sensor, float kP,  float kI, float kD, int threshold, int integralLimit, int iActiveZone, int slewRate, int rangeError, int tTarget) {
	pid *controller = gyroController->controller;
	initPIDController(controller, kP, kI, kD, threshold, integralLimit, slewRate);
	gyroController->sensor = sensor;
	gyroController->rangeError = rangeError;
	gyroController->tTarget = tTarget;
}

int getGyroCrossTrackError(int startHeading){
	float kp_gyro=0.2, ki_gyro=0.005;
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

bool drivePID(drivebase *controller, int heading = getGyroGlobal())
{
	resetEncOdo();
	pid *odo = controller->odo;
	int lastTime = nPgmTime;
	short timeAtTarget = 0;
	writeDebugStream("INTENDED: (O: %d) ", odo->target);
	//target for gcurve
	controller->startHeading = heading;
	gyroscope *gyroControllerCurve = gcurve; 
	pid *gyroPidCurve = gyroControllerCurve->controller;
	setTarget(gyroPidCurve, controller->startHeading);

	clearTimer(T2);
	clearTimer(T3);

	do{
    if (time1(T3) >= MOVE_TIMEOUT) { setWheelSpeed(0); writeDebugStreamLine("\nENDED FALSE"); controller->endHeading = getGyroGlobal(); return false; }

    lastTime = nPgmTime;

		setWheelSpeedTurn(
				limit(updatePIDController(odo, controller->odoEncoder), controller->maxSpeed), 
				limit(updatePIDController(gyroPidCurve, gyroControllerCurve->sensor), controller->maxSpeed)
		);

		writeDebugStreamLine("Err: %d Sens: %d", odo->error, SensorValue[controller->odoEncoder]);

		delay(10);
		
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

	writeDebugStreamLine("ACT: (%d)", SensorValue[controller->odoEncoder]);
	return true;
}

bool pointTurnGyroPID (gyroscope *gyroController) {
	resetAll();
	pid *controller = gyroController->controller;
	short timeAtTarget = 0;
	writeDebugStream("INTENDED: (%d) ", controller->target);
	clearTimer(T2);

	do {

		spin(limit(updatePIDController(controller, gyroController->sensor), gyroController->maxSpeed));

		writeDebugStreamLine("Err: %d Sens: %d", controller->error, getGyroGlobal());

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

	writeDebugStreamLine("ACT: (%d)", SensorValue[gyroController->sensor]);
	return true;
}

/* 
initPIDDrivebase (drivebase *controller, tSensors odoEncoder, tSensors gyroscope, float kP,  float kI, float kD, 
int threshold, int integralLimit, int iActiveZone, int slewRate, int rangeError, int tTarget) {

initPIDGyroscope (gyroscope *gyroController, tSensors sensor, float kP,  float kI, float kD, 
int threshold, int integralLimit, int iActiveZone, int slewRate, int rangeError, int tTarget) {
  */
/************************************************************************************/
/************************************** FNXs **************************************/
/************************************************************************************/
void initPids(void){
	initPIDDrivebase(db, sENCO, sGYRO,     0.285, 0, 1.5, 100, 30, -1, 50, 90, 250); //d=1.5
	initPIDDrivebase(dbload, sENCO, sGYRO, 0.15, 0, 1.5, 100, 30, 40, 50, 90, 225);
	initPIDGyroscope(g, sGYRO,             0.35, 0.002, 15, 100, 40, 40, 20, 90, 250); //d=15
	initPIDGyroscope(gload, sGYRO,         0.4, 0.002, 15, 100, 40, 40, 20, 90, 250);

  	initPIDDrivebase(dbfast, sENCO, sGYRO, 0.1, 0, 1.5, 100, 0, 40, 20, 90, 0);
	initPIDGyroscope(gfast, sGYRO,         0.2, 0, 0.7, 100, 0, 40, 20, 90, 200);
	initPIDGyroscope(gcurve, sGYRO,        0.24, 0.002, 1.5, 100, 0, 40, 20, 90, 0);
}

void autonDrive(drivebase *controller, float target, int maxSpeed, int wait, int heading=-1) {
	controller->maxSpeed = maxSpeed;
	setTarget(controller->odo, encoderCurveO(target));
	if(heading==-1)
		drivePID(controller);
	else drivePID(controller, heading);
	delay(wait);
}

void autonTurn (gyroscope *gyroController, float target, int maxSpeed, int wait) {
	gyroController->maxSpeed = maxSpeed;
	pid *controller = gyroController->controller;
	setTarget(controller, gyroCurve(target));
	pointTurnGyroPID(gyroController);
	delay(wait);
}

void autonDrive(float target, int maxSpeed, int wait, int heading=-1){ autonDrive(db, target, maxSpeed, wait, heading); }
void autonDriveLoad(float target, int maxSpeed, int wait){ autonDrive(dbload, target, maxSpeed, wait); }
void autonDriveFast(float target, int maxSpeed, int wait){ autonDrive(dbfast, target, maxSpeed, wait); }
void autonTurn(float target, int maxSpeed, int wait){ autonTurn(g, target, maxSpeed, wait); }
void autonTurnLoad(float target, int maxSpeed, int wait){ autonTurn(gload, target, maxSpeed, wait); }
void autonTurnFast(float target, int maxSpeed, int wait){ autonTurn(gfast, target, maxSpeed, wait); }

/************************************************************************************/
/************************************** LIFTs **************************************/
/************************************************************************************/
enum POSITION{IN, OUT, IDLE};
enum STATE{remainToBeDone, working, done, pause};
bool auto_mgoalin = false, auto_mgoalout = false, auto_clawout = false, auto_clawin = false, auto_armin = false, auto_armout = false; 

#define DEFAULT -99
#define OUT -1
#define IN -2
#define COUT -3
#define CIN -4
#define AOUT -5
#define AIN -6
#define _tick_mobilegoalout 2000
#define _tick_mobilegoalin 850

void putMobileGoalIn(void){
	setMGLiftMotor(0);
	setMGLiftMotor(-127);
	delay(100);
	do{ delay(50);
		if(motor[mMGLIFTR]==0) break; 
	}while(getSensorPotMGoal() > _tick_mobilegoalin);
	setMGLiftMotor(0);
}

void putMobileGoalOut(void){
	setMGLiftMotor(127);
	delay(100);
	do{ delay(50);
		if(motor[mMGLIFTR]==0) break; 
	}while(getSensorPotMGoal() < _tick_mobilegoalout);
	setMGLiftMotor(15); 
}

void putClawOut(void){
	pwrClaw(127);delay(450);pwrClaw(0);
}

void putClawIn(void){
	pwrClaw(-127);delay(300);pwrClaw(-30);
}

void putArmOut(void){
	pwrArm(127); delay(200); pwrClaw(-127); delay(200); pwrArm(0); 
}

void putArmIn(void){
	pwrClaw(-30); pwrArm(-127); delay(475); pwrArm(0); delay(350); pwrClaw(127); delay(300); pwrClaw(0);
}

task MobileGoalControl(){
	enum STATE stata = done;
	int aim = DEFAULT;
	while(true){
		if(auto_mgoalin){
			auto_mgoalin = false;
			aim = IN;
			stata = remainToBeDone;
		}
		if(auto_mgoalout){
			auto_mgoalout = false;
			aim = OUT;
			stata = remainToBeDone;
		}
		if(aim == IN && stata != done){
			putMobileGoalIn();
			stata = done;
		}else if(aim == OUT && stata != done){
			putMobileGoalOut();
			stata = done;
		}
		delay(20);
	}
}

task ClawControl(){
	enum STATE stata = done;
	int aim = DEFAULT; 
	while(true){
		if(auto_clawout){
			auto_clawout = false;
			aim = COUT;
			stata = remainToBeDone;
		}
		if(auto_clawin){
			auto_clawin = false; 
			aim = CIN;
			stata = remainToBeDone;
		}

		if(aim == COUT && stata != done){
			putClawOut();
			stata = done;
		}
		if(aim == CIN && stata != done){
			putClawIn();
			stata = done;
		}
		delay(20);
	}
}

task ArmControl(){
	enum STATE stata = done;
	int aim = DEFAULT;
	while(true){
		if(auto_armout){
			auto_armout = false;
			aim = AOUT;
			stata = remainToBeDone;
		}
		if(auto_armin){
			auto_armin = false; 
			aim = AIN;
			stata = remainToBeDone;
		}

		if(aim == AOUT && stata != done){
			putArmOut();
			stata = done;
		}
		if(aim == AIN && stata != done){
			putArmIn();
			stata = done;
		}
		delay(20);
	}
}

void autonMG(enum POSITION target){
	stopTask(MobileGoalControl);startTask(MobileGoalControl); 
	if(target==OUT) auto_mgoalout=true; 
	else if(target==IN) auto_mgoalin=true; 
}

void autonArm(enum POSITION target){
	stopTask(ArmControl);startTask(ArmControl); 
	if(target==OUT) auto_armout=true; 
	else if(target==IN) auto_armin=true; 
}

void autonClaw(enum POSITION target){
	stopTask(MobileGoalControl);startTask(ClawControl); 
	if(target==OUT) auto_clawout=true; 
	else if(target==IN) auto_clawin=true; 
}

#endif  // _AUTON_LIBRARY
