#include "main.h"
#include <stdio.h>
#include <stdbool.h>
using namespace pros::c;
using namespace pros;
using namespace std;

/**************************************************************************** 
	TIMER
***************************************************************************/ 
Timer::Timer(){ this->reset(); }
void Timer::reset(){ this->start = millis(); } 
long Timer::getTime(){ return millis()-(this->start); }


/**************************************************************************** 
	SLEW
***************************************************************************/ 
Slew_t::Slew_t(){
  slewRate=100; output=0;
  prevTime = millis();
};
double Slew_t::update(double in){
  int dt = millis() - prevTime;
  if (dt > 1000) dt = 0;
  prevTime = millis();
  double maxIncrease = slewRate * dt;
  double outputRate = (double)(in - output) / (double)dt;
  if (fabs(outputRate) < slewRate) output = in;
  else if (outputRate > 0) output += maxIncrease;
  else output -= maxIncrease;
  return output;
};


/**************************************************************************** 
	TBH
***************************************************************************/ 
bool TBH::isTbhInThresh(int thresh) { return abs(error) <= thresh; }
bool TBH::isTbhDerivInThresh(int thresh) { return abs(d) <= thresh; }
void TBH::resetIntegral() {
	this->sensVal = 0;
	d = i = p = 0;
}
void TBH::setTarget(int target){
	resetIntegral();
	this->target = target;
	this->integXing = 0;
	this->hasXed = false;
	// updateTbhErr();
	this->errXing = this->error; //Don't immediately trigger a crossing
	setTbhDoRun(true);
}
void TBH::setTbhDoRun(bool doRun) {
	this->doRun = doRun;
	if (this->doRun) this->doUpdate = true;
}
TBH::TBH(){}
TBH::TBH(double kP, double kI, double kD, int thresh, int integralLimit, bool doSgnLock, int slewRate){
  	slew.slewRate = slewRate;
	this->kP = kP; this->kI = kI; this->kD = kD; this->thresh = thresh; this->integralLimit = integralLimit; this->doSgnLock = doSgnLock;
	this->prevTime = 0;
	this->doUpdate = true;
	this->out = 0;
	this->sensVal = this->lastSensVal = this->lastD = 0;
	resetIntegral();
	setTarget(0);
	setTbhDoRun(false);
}
double TBH::update(){
	double dT = millis() - prevTime;
	if(dT>1000) dT=0;
	prevTime = millis();

	double _d = (sensVal - lastSensVal) / dT;

	sensVal = getVelF();
	if(sensVal < 0) sensVal = lastSensVal; 
	d = (sensVal + lastD) / 2.;
	lastSensVal = sensVal;
	lastD = _d;

	error = target - sensVal;
	isOnTgt = isTbhInThresh(thresh);
	if (isOnTgt) errThresh = 0;
	else errThresh = error - thresh * sgn(error);

	p = kP*error;

	if(!doRun){
		doUpdate = false;
		return out=0;
	}
	i = clamp((i + (errThresh*kI - d*kD)*dT), (double)-integralLimit, (double)integralLimit);

	if((errXing <= 0 && error > 0) || (errXing >= 0 && error < 0)){
		if(hasXed) i = (i+integXing) / 2.;
		else hasXed = false;
		integXing = i;
		errXing = error;
	}
	if(doSgnLock){
		if(sensVal > 0 && i < 0) i = 0, p=0;
		else if(sensVal < 0 && i > 0) i = 0, p=0;
	}
	// printf("TARG: %d Sens: %d OUT %f %f %f ERR: %d\n", target, sensVal, p, i, d, error);
	out = p+i; 
	return out;
}

/**************************************************************************** 
	PID
***************************************************************************/ 
PID::PID(){}
PID::PID(TYPE T, double kP, double kI, double kD, double unwind, double integralLimit, double slewRate, double dInactiveZone, double iActiveZone){
	slew.slewRate = slewRate;
    this->kP = kP; this->kI = kI; this->kD = kD; this->unwind = unwind; this->integralLimit = integralLimit; this->dInactiveZone = dInactiveZone; this->iActiveZone = iActiveZone;
    this->target = this->error = this->lastError = this->lastOutput = this->lastSensValue = this->errTot = this->lastDeriv = this->p = this->i = this->d = this->sensVal = 0;
	this->prevTime = this->prevDUpdateTime = 0;
	this->T = T;
}
double PID::update(){
	switch(T){
		case encoderL:
			sensVal = getDistance();
			break;
		case gyro:
			sensVal = getGyroGlobal();
			break;
		case integrated:
			sensVal = getArmC();  //UPDATED ON PER LOOP BASIS IN LOWER FUNCS
			break;
		case integrated2:
			sensVal = getPotA(); 
			break; 
		default: sensVal=0; break;
	}

	  error = target - sensVal; 
	  double dT = millis() - prevTime;
	  if(dT>1000) dT=0;
	  prevTime = millis();
	  p = kP*error;
	  d = lastDeriv;
	  double derivativeDt = millis() - prevDUpdateTime;

	  if(derivativeDt > 1000){
	    lastSensValue = sensVal;
	    prevDUpdateTime = millis();
	    d=0; //added reset
	  }else if(derivativeDt >= 15){
	    d = (lastError - error) * kD / derivativeDt;
	    prevDUpdateTime = millis();
	    lastDeriv = d;
	    lastSensValue = sensVal;
	  }
	  if(fabs(error) < dInactiveZone) d=0;
	  errTot += error * dT;
	  if(fabs(error) > iActiveZone) errTot = 0;
	  double maxErrTot = ((kI != 0.0) ? (integralLimit / kI) : 999999999);
	  errTot = clamp(errTot, -maxErrTot, maxErrTot); 

	  if((error > 0.0 && errTot < 0.0) || (error < 0.0 && errTot > 0.0) || fabs(error) < 0.001){
	  	if((fabs(error) - unwind) > -0.001) errTot = 0.0;
	  }
	  if(fabs(unwind) < 0.001 && fabs(error) < 0.001) errTot = 0;

	i = errTot * kI;
	i = (i >= integralLimit) ? 0 : i;

	double output = slew.update(p + i + d);
	printf("__PID__ T: %d Err: %d Sens: %d Spd: %d\n", (int)target, (int)error, (int)sensVal, (int)output);
	
	lastOutput = output;
	lastError = error;
	lastSensValue = sensVal;
	return output;
}


