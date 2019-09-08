#ifndef _CONFIG_H_
#define _CONFIG_H_
#include "main.h"
#include <stdbool.h>
using namespace pros::c;
using namespace pros;
using namespace std; 
#include <utility>
/*
INCLUDED IN MAIN.H
http://blog.olkie.com/2013/11/05/online-c-function-prototype-header-generator-tool/
*/

#define abs(X) ((X < 0) ? -1 * X : X)
#define neg(X) ((X > 0) ? -1 * X : X)
#define sgn(X) ((X >= 0) ? 1 : -1)
#define mem(a, b) memset(a, (b), sizeof(a))
#define len(x) (int)(sizeof(x)/sizeof(x[0]))
#define PI 3.14159265358979323846
#define endl '\n'
// #define TASK2(A, B) A, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, B
#define TASK(A) task_create(A, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "")

double fmaxf(double a, double b); 
double fminf(double a, double b); 
int limit (int num, int limit); 
double dlimit (double num, double limit); 
int clamp(int n, int min, int max); 
double clamp(double n, double min, double max); 
double encoderCurve (double target); 
double encoderCurveO (double target); 
double reverseEncoderCurveO (double target); 
double reverseEncoderCurve (double target); 
double gyroCurve (double target); 
double reverseGyroCurve(double target); 
double d2r(double target); 
double r2d(double target); 

#define mDRIVEL_1 1
#define mDRIVEL_2 2
#define mDRIVER_1 3
#define mDRIVER_2 4
#define mFLYU 9
#define mFLYD 10
#define mDS 6
#define mLIFT 7
#define mINTAKE 5
#define pGYRO 'V'
#define pPOTA 'B'
#define pPOTC 'A'
#define pENCL_1 'C' //L
#define pENCL_2 'D'
#define pENCR_1 'E' //L
#define pENCR_2 'F'
#define pENCF_1 'G' //F
#define pENCF_2 'H'
extern adi_encoder_t sENCF;
extern adi_encoder_t sENCL;
extern adi_encoder_t sENCR;
extern adi_gyro_t sGYRO;

enum STATE{remainToBeDone, working, done};
enum TYPE{encoderL, encoderE, gyro, integrated, integrated2};
enum LOCKMODE{LEFT, RIGHT}; 
enum TASKSELECTED{AUTO_DEFAULT, 
  STOP, S1, S2, S3, S3SLOW, S4, 
  DRIVE, TURN, SWEEP, SWEEP2, LOCK,
  POS, DRIVESLEW, DRIVENC
}; 

extern bool auto_stop, auto_S1, auto_S2, auto_S3, auto_S4, auto_S3SLOW; 
extern bool auto_drive, auto_driveslew, auto_driveNC, auto_turn, auto_sweep, auto_sweep2, auto_lock; 
extern bool auto_pos; 
extern bool doubleShot, cntON;
extern bool DISABLED;
extern int global_distance; 

#define MAX_CHOICE 4
#define MAX_CHOICE_LCD 3
extern int lcdTelemMode, autonomousMode, autonNumber;
extern bool mAutonBool;
extern bool global_flyR;
extern bool global_finishedT;


#define getEncF() (double)adi_encoder_get(sENCF)
#define getDL() (double)adi_encoder_get(sENCL) //odo w
#define getDR() (double)adi_encoder_get(sENCR) //odo w
#define getDistance() (getDR() - global_distance)
#define getDM() (double)((getDL()+getDR())*0.5)
#define getDI() (double)((motor_get_position(mDRIVEL_2) + motor_get_position(mDRIVEL_1))*0.5*1.4)
// #define getDR() (double)((-motor_get_position(mDRIVER_2) - motor_get_position(mDRIVER_1))*0.5)
#define getPotA() (double)adi_analog_read(pPOTA)
#define getArmC() (double)motor_get_position(mLIFT)

const double L_DISTANCE_IN = 3.25;
const double R_DISTANCE_IN = 3.25;
#define TURNWHEEL
#ifdef TURNWHEEL
  #define getGyroGlobal() (double)gyroCurve((getDL() - getDR())*1.68/(L_DISTANCE_IN + R_DISTANCE_IN))
#else
  #define getGyroGlobal() (double)adi_gyro_get(sGYRO)
#endif


#define resetDistance() global_distance=getDR()
#define resetEncF() adi_encoder_reset(sENCF)
#define resetEncL() adi_encoder_reset(sENCL)
#define resetEncR() adi_encoder_reset(sENCR)
#define resetDLR() motor_tare_position(mDRIVEL_1);motor_tare_position(mDRIVEL_2);motor_tare_position(mDRIVER_1);motor_tare_position(mDRIVER_2);
#ifdef TURNWHEEL
  #define resetGyro() adi_gyro_reset(sGYRO);
#else 
  #define resetGyro() resetDLR();
#endif
#define resetGB() resetGyro(); resetEncL(); resetEncR(); resetDLR(); 

//TASKS
extern task_t _displayInfoTask, _fwVelTask, _flyControlTask, _flywheelPID, _autonATask, _dsControlTask, _trackingTask;  
void flyControlTask(void *param);
void dsControlTask(void *param); 
void flywheelPID(void *param);
void autonATask(void *param); 
void autonDriveT(void *param);
void autonTurnT(void *param);
void fwVelTask(void *param);
void trackingTask(void *param); 
void reset_task_control();

 
void pwrFlywheel(int speed);
void pwrLift(int speed);
void pwrDS(int speed); 
void pwrIntake(int speed);
void pwrDriveLeft(int speed);
void pwrDriveRight(int speed);
void pwrDriveRightV(int speed); 
void pwrDriveLeftV(int speed);
void setWheelSpeedTurn(int speed, int turn);
void setWheelSpeed(int left, int right);
void setWheelSpeed(int speed);
void spin(int speed);
void lockDrive(LOCKMODE side); 
void unlockDrive(LOCKMODE side); 

//DRIVE FUNCS
void autonDrive(int target, int maxSpeed, int wait, int heading=-1); 
void autonDriveNC(int target, int maxSpeed, int wait);
void autonDriveSlew(int target, int maxSpeed, int wait, int heading=-1); 
void autonDriveSlewNC(int target, int maxSpeed, int wait); 
void autonSweep(int dist, pair<int, int> ARC, /*int angle, int startTurn,*/ int maxSpeed, int wait); 
void autonSweep(int dist, pair<int, int> ARC, pair<int, int> ARC2,/*int angle, int angle2, int startTurn, int startTurn2,*/ int maxSpeed, int wait); 
void autonTurn(int target, int maxSpeed, int wait); 
void autonTurnHP(int target, int maxSpeed, int wait); 
void autonLock(int target, LOCKMODE lockSide, int maxSpeed, int wait); 

void reachedError(double perc);
void waitFinish(); 

//FLY
int getVelF();
void waitV();
void waitDouble(); 
void setDriveMaxSpeed(int maxSpeed); 
void setDriveSlew(double slewRate); 
void setDriveTimeout(int MOVE_TIMEOUT); 
void setTurnTimeout(int MOVE_TIMEOUT); 

struct Slew_t {
  double slewRate, output;
  int prevTime;
  Slew_t();
  double update(double in);
};

struct Timer{
  long start; 
  Timer(); 
  void reset(); 
  long getTime(); 
}; 
extern Timer auton_timer; 

struct TBH{
  double p, d, i, kI, kD, kP;
  int error, integralLimit, errXing, errThresh, target, thresh, out, integXing, prevTime, sensVal, lastSensVal, lastD;
  bool doSgnLock, hasXed, doRun, doUpdate, isOnTgt;
  bool isTbhInThresh(int thresh);
  bool isTbhDerivInThresh(int thresh);
  Slew_t slew;
  void resetIntegral();
  void setTarget(int target);
  void setTbhDoRun(bool doRun);
  TBH();
  TBH(double kP, double kI, double kD, int thresh, int integralLimit, bool doSgnLock, int slewRate);
  double update();
};

struct PID {
  TYPE T;
  double kP, kI, kD, p, i, d;
  double target, error, lastError, integralLimit, slewRate, lastOutput, lastSensValue, errTot, iActiveZone, unwind, lastDeriv, sensVal, dInactiveZone;
  double customSensVal; 
  long prevTime, prevDUpdateTime;
  Slew_t slew;
  PID();
  PID(TYPE T, double kP, double kI, double kD, double unwind, double integralLimit, double slewRate, double dInactiveZone, double iActiveZone);
  double update();
};
void setTarget(PID &controller, int target); 
void setTarget(PID &controller, double target); 
extern PID lift; 

namespace DS{
  void move(int _target, int _pwr); 
  void reset(); 
}

namespace async{
  void autonDrive(int _dist, int _maxSpeed, int _delay, int _heading=-1); 
  void autonDriveSlew(int _dist, int _maxSpeed, int _delay, int _heading=-1); 
  void autonDriveNC(int _dist, int _maxSpeed, int _delay); 
  void autonTurn(int _angle, int _maxSpeed, int _delay);
  void autonSweep(int _dist, pair<int, int> _ARC, pair<int, int> _ARC2,/*int _angle, int _startTurn, int _angle2, int _startTurn2,*/ int _maxSpeed, int _delay); 
  void autonSweep(int _dist, pair<int, int> _ARC,/*int _angle, int _startTurn,*/ int _maxSpeed, int _delay); 
  void autonLock(int _angle, LOCKMODE _lockSide, int _maxSpeed, int _delay); 
}

namespace FLY{
  void set(int _rpm); 
  void doubleShot(int _startRpm, int _nextRpm, int _inSpeed, bool _intakeStop); 
}


//AUTONS
void autonFN(bool flip);
void autonFP(bool flip); 
void autonFar(bool flip); 
void autonFCounter(bool flip); 
void auton2(bool flip);
void auton3(bool flip);
void auton4(bool flip);
void autonSkills(); 
void autonSkills23();
void autonSkillsBack(); 
void odotest();

#endif
