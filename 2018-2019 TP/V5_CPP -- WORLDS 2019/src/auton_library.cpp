#include "main.h"
#include <stdio.h>
#include <stdbool.h>
using namespace pros::c;
using namespace pros;
using namespace std;
#include <vector>
#include <utility>

double fmaxf(double a, double b) { return a > b ? a : b; }
double fminf(double a, double b) { return a < b ? a : b; }
int limit (int num, int limit) { return sgn(num)*(abs(num)>limit?limit:abs(num)); }
double dlimit (double num, double limit) { return sgn(num)*(fabs(num)>limit?limit:fabs(num)); }
int clamp(int n, int min, int max) { return n < min ? min : (n > max ? max : n); }
double clamp(double n, double min, double max) { return n < min ? min : (n > max ? max : n); }
double r2d(double target){ return target*(180.0/PI); }
double d2r(double target){ return target*(PI/180.0); }
double encoderCurve (double target) { return 27.2*target * 1.5; }
double encoderCurveO (double target) {return 35.3*target * 1.5; }
double reverseEncoderCurveO (double target) {return target/35.3; }
double reverseEncoderCurve (double target) {return target/(27.2*1.75); }
double gyroCurve (double target) { return 10.5*target; }
double reverseGyroCurve(double target) { return target/10.5; }

#define VELMODE //for drive/fly
#ifdef NOVELMODE
	void pwrFlywheel(int speed){ //600 cart
		speed = 600*fabs(speed/127.0) * sgn(speed);
		motor_move_velocity(mFLYU, -speed); motor_move_velocity(mFLYD, speed);
	}
#else
	void pwrFlywheel(int speed){motor_move(mFLYU, -speed); motor_move(mFLYD, speed);}
#endif

void pwrIntake(int speed){motor_move(mINTAKE, speed);}
void pwrLift(int speed){motor_move(mLIFT, -speed);}
void pwrDS(int speed){motor_move(mDS, speed);}
void pwrDriveRightNoVel(int speed){motor_move(mDRIVER_1, speed); motor_move(mDRIVER_2, speed);}
void pwrDriveLeftNoVel(int speed){motor_move(mDRIVEL_1, -speed); motor_move(mDRIVEL_2, -speed);}
void pwrDriveRightV(int speed){ //gearset +-100, 200, 600
	speed = 200*fabs(speed/127.0) * sgn(speed);
	motor_move_velocity(mDRIVER_1, speed); motor_move_velocity(mDRIVER_2, speed);
}
void pwrDriveLeftV(int speed){
	speed = 200*fabs(speed/127.0) * sgn(speed);
	motor_move_velocity(mDRIVEL_1, -speed); motor_move_velocity(mDRIVEL_2, -speed);
}
void pwrDriveRight(int speed){motor_move(mDRIVER_1, speed); motor_move(mDRIVER_2, speed);}
void pwrDriveLeft(int speed){motor_move(mDRIVEL_1, -speed); motor_move(mDRIVEL_2, -speed);}

#ifdef NOVELMODE
	void spin(int speed){pwrDriveRightV(speed); pwrDriveLeftV(-speed); }
#else 
	void spin(int speed){pwrDriveRight(speed); pwrDriveLeft(-speed); }	
#endif
void setWheelSpeed(int left, int right){pwrDriveRight(left); pwrDriveLeft(right);}
void setWheelSpeed(int spd){setWheelSpeed(spd, spd);}
void setWheelSpeed(double left, double right){pwrDriveRight(left); pwrDriveLeft(right);}
void setWheelSpeed(double spd){setWheelSpeed(spd, spd);}
void setWheelSpeedTurn(int speed, int turn){setWheelSpeed(speed+turn, speed-turn);}
void lockDrive(LOCKMODE side){motor_set_brake_mode(side==RIGHT ? mDRIVER_1 : mDRIVEL_1, E_MOTOR_BRAKE_HOLD); motor_set_brake_mode(side==RIGHT ? mDRIVER_2 : mDRIVEL_2, E_MOTOR_BRAKE_HOLD ); }
void unlockDrive(LOCKMODE side){motor_set_brake_mode(side==RIGHT ? mDRIVER_1 : mDRIVEL_1, E_MOTOR_BRAKE_COAST); motor_set_brake_mode(side==RIGHT ? mDRIVER_2 : mDRIVEL_2, E_MOTOR_BRAKE_COAST); }

bool auto_stop, auto_S1, auto_S2, auto_S3, auto_S4, auto_S3SLOW; 
bool auto_drive, auto_driveNC, auto_driveslew, auto_turn, auto_sweep, auto_sweep2, auto_lock; 
bool auto_pos; 
bool global_flyR, global_finishedT; 
bool doubleShot; 

// /************************************************************************************/
// /************************************** STRUCTs **************************************/
// /************************************************************************************/

int velocities[5];
long lastdt;
void fwVelTask(void *param){
	mem(velocities, 0);
	int nextidx=0;
	lastdt = millis();
	while(true){
		long tme=millis();
		velocities[nextidx]=(((double)-getEncF())/360)/(((double)(tme-lastdt)==0?1:(double)(tme-lastdt)/(double)60)/1000);
		resetEncF();
		if(++nextidx==5) nextidx=0;
		lastdt=tme;
		delay(5); //5
	}
}

int getVelF(){
	long sum=0;
	for(int i=0;i<5;++i) sum+=velocities[i];
	return (int)(sum/5);
}

struct FLYWHEEL{
	TBH pFly;
	int maxSpeed, minSpeed, rangeError;
	FLYWHEEL(){}
	FLYWHEEL(TBH &pFly, int rangeError){
		this->pFly = pFly; this->rangeError = rangeError;
	}
};

struct DRIVEBASE{
	PID pDrive, pCurve;
	int maxSpeed; //ON-CALL
	int rangeError, tTarget, startHeading, endHeading;
	int MOVE_TIMEOUT; 
	DRIVEBASE(PID &pDrive, PID &pCurve, int rangeError, int tTarget){
		this->pDrive = pDrive; this->pCurve = pCurve;
		this->rangeError = rangeError;
		this->tTarget = tTarget;
		this->MOVE_TIMEOUT = 750; 
	}
};

struct GYROSCOPE{
	PID pGyro;
	int maxSpeed; //ON-CALL
	int rangeError, tTarget, startHeading, endHeading;
	int MOVE_TIMEOUT; 
	GYROSCOPE(PID &pGyro, int rangeError, int tTarget){
		this->pGyro = pGyro;
		this->rangeError = rangeError;
		this->tTarget = tTarget;
		this->MOVE_TIMEOUT = 750; 
	}
};

namespace internalData{
	int target, error; 
}

/**************************************************************************** 
	PID(TYPE T, double kP, double kI, double kD, double unwind, double integralLimit, double slewRate, double dInactiveZone, double iActiveZone)
	TBH(double kP, double kI, double kD, int thresh, int integralLimit, bool doSgnLock, int slewRate)
***************************************************************************/ 
TBH flywheelHigh(0.2, 8, 127, 0, 1, true, 50); //HIGHER RPM //.2, 0
TBH flywheelLow(1.6, 0, 100, 0, 1, true, 50); //LOWER RPM

#define SLEWRATE 1
PID driveRegular(encoderL, 0.27, 0, 8, 0, 9999999, 40, 0, 40);
PID driveBigSlew(encoderL, 0.32/*13*/, 0, 7, 0, 9999999, 0.4, 0, 40);
PID curve(gyro, 0.2, 0, 5, 0, 9999999, 40, 0, 40); //.37
PID gyroRegular(gyro, 0.22, 0, 6, 0, 9999999, SLEWRATE, 0, 40); 
PID gyroLock(gyro, 0.4, 0, 6, 0, 9999999, 5, 0, 40); 
PID gyroHP(gyro, 0.28, 0, 6, 0, 9999999, 1.4, 0, 40);
// PID gyroNoOcc(gyro, 0.15, 0/*.001*/, 3, 15, 9999999, SLEWRATE, 0, 40); 
// PID gyroHP(gyro, 0.45, 0, 8, 15, 9999999, 40, 0, 40);   
PID lift(integrated, 0.3, 0, 0, 15, 9999999, 50, 0, 40);
PID arm(integrated2, 0.06, 0, 2, 15, 9999999, 50, 0, 40);

DRIVEBASE db(driveRegular, curve, 40, 250);//250
DRIVEBASE dbSlew(driveBigSlew, curve, 100, 250);//250
GYROSCOPE g(gyroRegular, gyroCurve(3.75), 350);
GYROSCOPE gLock(gyroLock, gyroCurve(3.5), 350);
GYROSCOPE gHP(gyroHP, gyroCurve(1.5), 350);

FLYWHEEL f(flywheelLow, /*400*/100);

/************************************************************************************/
/************************************** INITs **************************************/
/************************************************************************************/

void clearIntegral(PID &controller) {
	controller.i = 0;
}

void setTarget(PID &controller, int target) {
	clearIntegral(controller);
	controller.target = target;
}

void setTarget(PID &controller, double target) {
	clearIntegral(controller);
	controller.target = target;
}

void setCustSensVal(PID &controller, double customSensVal) { //SET TARGET 0
	controller.target = 0.0; 
	controller.customSensVal = customSensVal;
}

void setDriveMaxSpeed(int maxSpeed){
	db.maxSpeed = maxSpeed; 
}

void setDriveSlew(double slewRate){
	driveBigSlew.slewRate = slewRate; 
}

void setDriveTimeout(int MOVE_TIMEOUT){ //also works for sweep
	db.MOVE_TIMEOUT = MOVE_TIMEOUT; 
}

void setTurnTimeout(int MOVE_TIMEOUT){
	g.MOVE_TIMEOUT = MOVE_TIMEOUT; 
}

void reachedError(double perc){
	double cur; 
	do{
		delay(50);
		cur = ((abs(internalData::target)-abs(internalData::error)) / (double)abs(internalData::target)); 
	}while(cur < perc); 
}

/************************************************************************************/
/************************************** PIDs **************************************/
/************************************************************************************/
#define IMPACT
bool drivePID(DRIVEBASE &controller, int heading=-1, int heading2=-1, bool mgoal=false, int startTurn=0, int startTurn2=0, bool correctionON=true)
{
	//switch for correct pDrive
	// PID &pDrive = (heading != -1 ? (heading>0 ? controller.pDriveL : controller.pDriveR) : controller.pDriveL); 
	PID &pDrive = controller.pDrive; 

	resetDistance(); 
	short timeAtTarget = 0;

	controller.startHeading = getGyroGlobal();
	PID &pCurve = controller.pCurve;
	setTarget(pCurve, controller.startHeading);

	printf("INTENDED: (O: %d) (G: %d) \n", (int)pDrive.target, controller.startHeading);

	Timer T2, T3; 

	bool mgoalstarted = false, mgoalstarted2 = false, recordError = false;
	int startError = pDrive.error; //0
	int prevError = pDrive.error; //error>0 on start
	do{
		if(correctionON)
  			setWheelSpeedTurn(
	  			-limit(pDrive.update(), controller.maxSpeed),
  				limit(pCurve.update(), controller.maxSpeed)
  			);
		else 
			setWheelSpeed(
	  			-limit(pDrive.update(), controller.maxSpeed)
  			);

		#ifdef IMPACT
			if(abs(prevError-abs(pDrive.error)) > 2) T3.reset();
			if (T3.getTime() >= controller.MOVE_TIMEOUT) { setWheelSpeed(0, 0); printf("\nENDED FALSE"); controller.endHeading = getGyroGlobal(); return false; }
			prevError = abs(pDrive.error);
		#endif

		if(!recordError){ startError = pDrive.error; recordError=true; }

		printf("Err: %d Sens: %d GErr: %d GSens: %d TOUT: %lu ST: %f\n", (int)pDrive.error, (int)pDrive.sensVal, (int)pCurve.error, (int)(pCurve.sensVal), T3.getTime(), startTurn);
		internalData::target = pDrive.target; 
		internalData::error = pDrive.error; 
		delay(10);

		if(!mgoalstarted && mgoal && abs(reverseEncoderCurveO(abs(startError)-abs(pDrive.error)))>startTurn){
      		controller.startHeading += heading;
      		setTarget(pCurve, controller.startHeading);
			printf("STARTED CURVE");
			mgoalstarted=true;
		}

		if(!mgoalstarted2 && mgoal && (startTurn2>0 && abs(reverseEncoderCurveO(abs(startError)-abs(pDrive.error)))>startTurn2)){
      		controller.startHeading += heading2;
      		setTarget(pCurve, controller.startHeading);
			printf("STARTED CURVE");
			mgoalstarted2=true;
		}

    	if(abs(pDrive.error) <= controller.rangeError && abs(pCurve.error)<=controller.rangeError) {//testing curve
			timeAtTarget = T2.getTime();
			if(timeAtTarget >= controller.tTarget) {
				setWheelSpeed(0);
				controller.endHeading = getGyroGlobal();
				printf("\nENDED TRUE");
				return true;
			}
		}
		else T2.reset(); 

		delay(10);
	} while(true);

	//printf("ACT: (%d)", SensorValue[controller.odoEncoder]);
	return true;
}

bool pointTurnGyroPID (GYROSCOPE &gyroController)
{
	resetGB();
	PID &pGyro = gyroController.pGyro;
	short timeAtTarget = 0;
	printf("INTENDED: (%d) ", pGyro.target);

	Timer T1; 
	do {
		//if (time1(T3) >= gyroController.MOVE_TIMEOUT) { setWheelSpeed(0); printf("\nENDED FALSE"); return false; }

		int pwrout = (int)limit(pGyro.update(), gyroController.maxSpeed); 
		if(pwrout!=0)
			spin(
				pwrout
			);
		else setWheelSpeed(0); 

		// printf("Err: %d Sens: %d\n", pGyro.error, pGyro.sensVal);

		delay(10);
		if(abs(pGyro.error) <= gyroController.rangeError) {
			timeAtTarget = T1.getTime();
			if(timeAtTarget >= gyroController.tTarget) {
				setWheelSpeed(0);
	    		printf("\nENDED TRUE");
				return true;
			}
		}
		else T1.reset(); 

		delay(10);

	} while(true);

	return true;
}

bool lockTurnGyroPID(GYROSCOPE &gyroController, LOCKMODE side)
{
	resetGB();
	PID &pGyro = gyroController.pGyro;
	short timeAtTarget = 0;
	printf("INTENDED: (%d) ", pGyro.target);

	Timer T1; 

	lockDrive(side); 

	do {
		//if (time1(T3) >= gyroController.MOVE_TIMEOUT) { setWheelSpeed(0); printf("\nENDED FALSE"); return false; }

		if(side==RIGHT){
			pwrDriveLeft(-limit(pGyro.update(), gyroController.maxSpeed)); 
		}
		else
			pwrDriveRight(limit(pGyro.update(), gyroController.maxSpeed)); 

		printf("Err: %d Sens: %d\n", pGyro.error, pGyro.sensVal);

		delay(10);
		if(abs(pGyro.error) <= gyroController.rangeError) {
			timeAtTarget = T1.getTime();
			if(timeAtTarget >= gyroController.tTarget) {
				setWheelSpeed(0);
				unlockDrive(side==LEFT ? RIGHT : LEFT); 
	    		printf("\nENDED TRUE");
				return true;
			}
		}
		else T1.reset(); 

		delay(10);

	} while(true);

	return true;
}

void flywheelPID(void *param)
{
	FLYWHEEL &flyController = f;
	TBH &pFly = flyController.pFly;

	printf("INTENDED F: (T: %d)\n", pFly.target);

	global_flyR=false;
	do{
		// pFly.update();
		int pwr = abs(dlimit(pFly.update(), flyController.maxSpeed));
		pwrFlywheel(pwr);
		// if(f.pFly.doRun)
		// printf("FWPID| T: %d RPM: %d RAW SPD: %d Error: %d\n", pFly.target, pFly.sensVal, abs(output), pFly.error);

		delay(10);
		if(pFly.error <= flyController.rangeError && pFly.error >= -flyController.rangeError){
			global_flyR=true;
		}else global_flyR = false; 

		printf("T: %d Err: %d Sens: %d Spd: %d GF:%d\n", pFly.target, pFly.error, getVelF(), pwr, global_flyR);

		delay(10);
	} while(true);
}

/************************************************************************************/
/************************************** FNXs ****************************************/
/************************************************************************************/
void autonDrive(DRIVEBASE &controller, int target, int maxSpeed, int wait, int heading=-1, bool correctionOn=true) {
	controller.maxSpeed = maxSpeed;
	setTarget(controller.pDrive, encoderCurveO(target));
	if(correctionOn){
		if(heading==-1) drivePID(controller);
		else drivePID(controller, gyroCurve(heading));
	}else{
		drivePID(controller, -1, -1, false, 0, 0, false);
	}
	delay(wait);
}
void autonTurn(GYROSCOPE &gyroController, int target, int maxSpeed, int wait) {
	gyroController.maxSpeed = maxSpeed;
	setTarget(gyroController.pGyro, gyroCurve(target));
	pointTurnGyroPID(gyroController);
	delay(wait);
}
void autonSweep(DRIVEBASE &controller, int dist, int angle, int angle2, int startTurn, int startTurn2, int maxSpeed, int wait){
  	controller.maxSpeed = maxSpeed;
	setTarget(controller.pDrive, encoderCurveO(dist));
	drivePID(controller, gyroCurve(angle), gyroCurve(angle2), true, startTurn, startTurn2);	
	delay(wait);
}

void autonLock(GYROSCOPE &gyroController, int target, LOCKMODE lockSide, int maxSpeed, int wait) {
	gyroController.maxSpeed = maxSpeed;
	setTarget(gyroController.pGyro, gyroCurve(target));
	lockTurnGyroPID(gyroController, lockSide);
	delay(wait);
}

task_t _flywheelPID;
void startFlywheel(int target, int maxSpeed, int minSpeed){
  	global_flyR=false;
	FLYWHEEL &flyController = f;
	flyController.maxSpeed=maxSpeed;
	flyController.minSpeed=minSpeed;
	flyController.pFly.setTarget(target); //also resets INT
	while(getVelF()<-30) delay(50);
	if(_flywheelPID == NULL)
	 	_flywheelPID  = TASK(flywheelPID);
}

void stopFlywheel(){
	// task_suspend(fwPID); //GLOBAL MISMATCH
	f.pFly.setTbhDoRun(false);
	pwrFlywheel(0);
}

void waitV(){
	do{delay(50);}while(!global_flyR);
}

void waitFinish(){
	do{delay(50);}while(!global_finishedT); 
}

void waitDouble(){
	Timer T1; 
	do{delay(50);}while(doubleShot && T1.getTime()<1200);  //doubleshot timeout
}

void autonDrive(int target, int maxSpeed, int wait, int heading){ autonDrive(db, target, maxSpeed, wait, heading); } //DEF HEADER
void autonDriveNC(int target, int maxSpeed, int wait){ autonDrive(db, target, maxSpeed, wait, -1, false); } //DEF HEADER
void autonDriveSlew(int target, int maxSpeed, int wait, int heading){ autonDrive(dbSlew, target, maxSpeed, wait, heading); } //DEF HEADER
void autonDriveSlewNC(int target, int maxSpeed, int wait){ autonDrive(dbSlew, target, maxSpeed, -1, false); } //DEF HEADER
void autonSweep(int dist, pair<int, int> ARC, int maxSpeed, int wait){ 
	autonSweep(db, dist, ARC.first, 0, ARC.second, 0, maxSpeed, wait);
}
void autonSweep(int dist, pair<int, int> ARC, pair<int, int> ARC2,/*int angle, int startTurn, int angle2, int startTurn2,*/ int maxSpeed, int wait){ 
	autonSweep(db, dist, ARC.first, ARC2.first, ARC.second, ARC2.second, maxSpeed, wait); 
}
void autonTurn(int target, int maxSpeed, int wait){ autonTurn(g, target, maxSpeed, wait);}
void autonTurnHP(int target, int maxSpeed, int wait){ autonTurn(gHP, target, maxSpeed, wait); }
void autonLock(int target, LOCKMODE lockSide, int maxSpeed, int wait){ autonLock(gLock, target, lockSide, maxSpeed, wait); }

/************************************************************************************/
/************************************** FLYWHEEL ************************************/
/************************************************************************************/

namespace FLY{
	int rpm, startRpm, nextRpm, inSpeed; 
	bool intakeStop;
	void set(int _rpm){
		rpm = _rpm; 
		auto_S4 = true; 
	}
	void doubleShot(int _startRpm, int _nextRpm, int _inSpeed, bool _intakeStop){ //+ inspd
		startRpm = _startRpm; nextRpm = _nextRpm; inSpeed = _inSpeed; intakeStop = _intakeStop; 
		auto_S3SLOW = true; 
	}
}

bool cntON = false; 
void flyControlTask(void *param){
	enum STATE stata = done;
	enum TASKSELECTED aim = AUTO_DEFAULT;  

  	Timer T1; 
	while(true){
		if(T1.getTime() > 300){
			if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_X) || auto_S1){
				auto_S1 = false;
				aim = S1; stata = remainToBeDone;
			}
			if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_Y) || auto_S2){
				auto_S2 = false;
				aim = S2; stata = remainToBeDone;
			}
			if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_A) || auto_S3){
				auto_S3 = false;
				aim = S3; stata = remainToBeDone;
			}
			if(auto_S3SLOW){
				auto_S3SLOW = false;
				aim = S3SLOW; stata = remainToBeDone;
			}
			if(auto_S4){
				auto_S4 = false;
				aim = S4; stata = remainToBeDone;
			}
			if(controller_get_digital(CONTROLLER_MASTER, DIGITAL_B) || auto_stop){
				auto_stop = false;
				aim = STOP; stata = remainToBeDone;
			}
		}
		if(stata == remainToBeDone) T1.reset(); 

		if(aim == S1 && stata != done){
			doubleShot=false; 
			f.pFly.resetIntegral();
			startFlywheel(/*2700*/2600, 127, 40);
			stata=done;
		}
		if(aim == S2 && stata != done){
			doubleShot=false; 
			f.pFly.resetIntegral(); 
			startFlywheel(/*2300*/2100, 127, 40);
			stata=done;
		}
		if(aim == S3 && stata != done){ //driver
			doubleShot=true;
			pwrIntake(-105);
			do{delay(25);} while(!controller_get_digital(CONTROLLER_MASTER, DIGITAL_B) && (/*2700*/2400 - abs(getVelF())) < 325); 
			startFlywheel(1800, 127, 40); //1900
			do{delay(25);} while(!controller_get_digital(CONTROLLER_MASTER, DIGITAL_B) && (1950 - abs(getVelF())) < 325); 
			pwrIntake(0);
			delay(50); 
			startFlywheel(/*2700*/2400, 127, 40); 
			doubleShot=false;  
			stata=done;
		}
		if(aim == S3SLOW && stata != done){ //auto
			doubleShot=true; cntON = false; 
			pwrIntake(-FLY::inSpeed); 
			do{delay(25);} while(!controller_get_digital(CONTROLLER_MASTER, DIGITAL_B) && (f.pFly.target - abs(getVelF())) < 400 && !cntON); 
			startFlywheel(FLY::nextRpm, 127, 40);
			do{delay(25);} while(!controller_get_digital(CONTROLLER_MASTER, DIGITAL_B) && (FLY::nextRpm - abs(getVelF())) < 150 && !cntON); 
			if(FLY::intakeStop) pwrIntake(0);
			delay(50); 
			startFlywheel(FLY::startRpm, 127, 40); 
			doubleShot=false;  
			stata=done;
		}
		if(aim == S4 && stata != done){
			doubleShot=false; 
			f.pFly.resetIntegral();
			startFlywheel(FLY::rpm, 127, 40);
			stata=done;
		}
		else if(aim == STOP && stata != done){
			// doubleShot=false; 
			stopFlywheel();
			stata=done;
		}
		delay(20);
  }
}

namespace async{
	//D/T/S
	int dist, angle, delay, maxSpeed, heading; 
	pair<int, int> ARC; 
	//S
	int startTurn, angle2, startTurn2; 
	pair<int, int> ARC2; 
	//L
	LOCKMODE lockSide;
	void autonDrive(int _dist, int _maxSpeed, int _delay, int _heading){
		global_finishedT=false;
		dist=_dist; maxSpeed=_maxSpeed; delay=_delay; heading=_heading; 
		auto_drive=true; 
	}
	void autonDriveNC(int _dist, int _maxSpeed, int _delay){
		global_finishedT=false;
		dist=_dist; maxSpeed=_maxSpeed; delay=_delay;
		auto_driveNC=true; 
	}
	void autonDriveSlew(int _dist, int _maxSpeed, int _delay, int _heading){
		global_finishedT=false;
		dist=_dist; maxSpeed=_maxSpeed; delay=_delay; heading=_heading; 
		auto_driveslew=true; 
	}
	void autonTurn(int _angle, int _maxSpeed, int _delay){
		global_finishedT=false; 
		angle=_angle; maxSpeed=_maxSpeed; delay=_delay;
		auto_turn=true;
	}
	void autonSweep(int _dist, pair<int, int> _ARC, pair<int, int> _ARC2,/*int _angle, int _startTurn, int _angle2, int _startTurn2,*/ int _maxSpeed, int _delay){
		global_finishedT=false; 
		dist=_dist; ARC=_ARC; ARC2=_ARC2;/*angle=_angle; startTurn=_startTurn; angle2=_angle2; startTurn2=_startTurn2;*/ maxSpeed=_maxSpeed; delay=_delay; 
		auto_sweep=true;
	}
	void autonSweep(int _dist, pair<int, int> _ARC,/*int _angle, int _startTurn,*/ int _maxSpeed, int _delay){
		global_finishedT=false; 
		dist=_dist; ARC=_ARC; /*angle=_angle; startTurn=_startTurn;*/ maxSpeed=_maxSpeed; delay=_delay; 
		auto_sweep2=true;
	}
	void autonLock(int _angle, LOCKMODE _lockSide, int _maxSpeed, int _delay){
		global_finishedT=false; 
		angle=_angle; lockSide=_lockSide; maxSpeed=_maxSpeed; delay=_delay; 
		auto_lock=true; 
	}
}

void autonATask(void *param){
	enum STATE stata = done;
	enum TASKSELECTED aim = AUTO_DEFAULT;  

  	Timer T1; 
	while(true){
		if(T1.getTime() > 300){
			if(auto_drive){
				auto_drive = false;
				aim = DRIVE; stata = remainToBeDone;
			}
			if(auto_driveNC){
				auto_driveNC = false;
				aim = DRIVENC; stata = remainToBeDone;
			}
			if(auto_driveslew){
				auto_driveslew = false;
				aim = DRIVESLEW; stata = remainToBeDone;
			}
			if(auto_turn){
				auto_turn = false;
				aim = TURN; stata = remainToBeDone;
			}
			if(auto_sweep){
				auto_sweep = false;
				aim = SWEEP; stata = remainToBeDone;
			}
			if(auto_sweep2){
				auto_sweep2 = false;
				aim = SWEEP2; stata = remainToBeDone;
			}
			if(auto_lock){
				auto_lock = false;
				aim = LOCK; stata = remainToBeDone;
			}
		}
		if(stata == remainToBeDone) T1.reset(); 

		if(aim == DRIVE && stata != done){
			global_finishedT=false;
			autonDrive(async::dist, async::maxSpeed, async::delay, async::heading); 
			global_finishedT=true;
			stata=done;
		}
		if(aim == DRIVENC && stata != done){
			global_finishedT=false;
			autonDriveNC(async::dist, async::maxSpeed, async::delay); 
			global_finishedT=true;
			stata=done;
		}
		if(aim == DRIVESLEW && stata != done){
			global_finishedT=false;
			autonDriveSlew(async::dist, async::maxSpeed, async::delay, async::heading); 
			global_finishedT=true;
			stata=done;
		}
		if(aim == TURN && stata != done){
			global_finishedT=false;
			autonTurn(async::angle, async::maxSpeed, async::delay); 
			global_finishedT=true;
			stata=done;
		}
		if(aim == SWEEP && stata != done){
			global_finishedT=false;
			autonSweep(async::dist, async::ARC, async::ARC2,/*async::angle, async::startTurn, async::angle2, async::startTurn2,*/ async::maxSpeed, async::delay); 
			global_finishedT=true;
			stata=done;
		}
		if(aim == SWEEP2 && stata != done){
			global_finishedT=false;
			autonSweep(async::dist, async::ARC,/*async::angle, async::startTurn,*/ async::maxSpeed, async::delay); 
			global_finishedT=true;
			stata=done;
		}
		if(aim == LOCK && stata != done){
			global_finishedT=false;
			autonLock(async::angle, async::lockSide, async::maxSpeed, async::delay); 
			global_finishedT=true;
			stata=done;
		}
		delay(20);
  }
}

#define OFFSET 275
namespace DS{
	int target, pwr; 
	bool rDS; 
	void move(int _target, int _pwr){
		target = _target-OFFSET; pwr = _pwr; //reversed
		rDS = false; 
		auto_pos = true; 
	}
	void reset(){rDS=true;}
}


void dsControlTask(void *param){
	enum STATE stata = done;
	enum TASKSELECTED aim = AUTO_DEFAULT;  

  	Timer T1; 
	while(true){
		if(T1.getTime() > 300){
			if(auto_pos){
				auto_pos = false;
				aim = POS; stata = remainToBeDone;
			}
		}
		if(stata == remainToBeDone) T1.reset(); 

		if(aim == POS && competition_is_autonomous() && !DS::rDS/*&& stata != done*/){
			stata = working;  
			motor_set_brake_mode(mDS, E_MOTOR_BRAKE_HOLD); 
			setTarget(arm, DS::target); 
			double pwr = limit(arm.update(), 127); 
			cout << "ERR: " << arm.error << endl; 
			pwrDS(pwr);

			// motor_set_brake_mode(mDS, E_MOTOR_BRAKE_HOLD);
			// int err = (DS::target - getPotA()), prevError = err; 
			// DS::pwr *= (err<0 ? -1 : 1); 
			// motor_move(mDS, DS::pwr); 
			// delay(50); 
			// if(err < 0){
			// 	while(err < 0 /*|| abs(err)>50*/){
			// 		// if((abs(prevError-abs(err)))<10) break;  //REMOVE for stalling
			// 		prevError = err; 
			// 		err = DS::target - getPotA();
			// 		cout << "T: " << DS::target << " E: " << err << endl; 
			// 		delay(5); 
			// 	}
			// }else if(err > 0){
			// 	while(err > 0 /*|| abs(err)>50*/){
			// 		// if((abs(prevError-abs(err)))<5) break;  //REMOVE for stalling
			// 		prevError = err; 
			// 		err = DS::target - getPotA();
			// 		cout << "T: " << DS::target << " E: " << err << endl; 
			// 		delay(5); 
			// 	}
			// }
			// motor_move(mDS, 0); 
			// stata=done;
		}else if(competition_is_autonomous()) pwrDS(0); 
  		delay(20);
  }
}

