#include "main.h"
#include <stdio.h>
#include <stdbool.h>
using namespace std; 
using namespace pros::c;
using namespace pros;

void odotest(){
	// autonDrive(-10, 100, 50);
	// autonTurn(90, 90, 0); 

	// async::autonDrive(35, 70, 0);
	// reachedError(.2);
	// DS::move(1000, 100);
	// reachedError(.5);
	// setDriveMaxSpeed(110); 
	// waitFinish();
	// reachedError(.4); 
	// setDriveMaxSpeed(110);
	// waitFinish();   



	/*setDriveTimeout(9999); 
	autonDriveNC(41, 100, 0); 
	setDriveTimeout(450); */
}

void autonFP(bool flip){
	float x = getGyroGlobal(); 
	async::autonDrive(25, 100, 50);
	reachedError(.2); 
	FLY::set(2400);
	reachedError(.4); 
	pwrIntake(127);
	waitFinish();

	setDriveTimeout(150);
	// autonDrive(-34, 100, 50);
	resetGB(); 
	async::autonDriveSlew(flip?-28:-28, /*{8, 11},*/ 100, 200, x); 
		reachedError(.5); 
		setDriveSlew(5); 
	waitFinish(); 
	setDriveSlew(0.4); 
	setDriveTimeout(450);

	// X: 
	// autonDrive(4, 100, 0); 
	async::autonTurn(flip?81:-78, 80, 25);
		reachedError(.5); 
		pwrIntake(0); 
	waitFinish(); 
	// FLY::doubleShot(2400, 1790, 105, true);  
	// waitDouble();
	// delay(50);
	pwrIntake(-127);
	delay(260);
 	pwrIntake(0);
	delay(125);
	pwrIntake(127);
	delay(150);
	FLY::set(2400);
	// async::autonSweep(flip?26:27, {flip?13:-12, flip?13:14}, 80, 200);
	x = getGyroGlobal(); 
	async::autonSweep(flip?24:26, {flip?15:-13, flip?20:21}, 80, 0/*, x*/); 
		pwrIntake(0);
		// auto_S2=true; //S3
		reachedError(.28);
		pwrIntake(-127);
		reachedError(.95);
		DS::move(3000, 127);
	waitFinish();
	// autonTurn(15, 110, 0); 
	// DS::move(3000, 127); 
	delay(250); 
	pwrIntake(0); 
	FLY::set(2400);
	resetGB();  //reset GYRO

	DS::move(400, 100); 

	autonSweep(-32, {flip?-15:15, 3}, 100, 50);
	setDriveTimeout(175); 
	autonSweep(-18, {flip?-85:85, 0}, 60, 100); 
	setDriveTimeout(450); 
	// autonTurn(89, 100, 0); 
	autonDrive(8, 60, 0);
	setDriveTimeout(9999);
	async::autonDriveNC(19, 108, 0); 
		reachedError(.2);
		pwrIntake(0); 
		reachedError(.4); 
		pwrIntake(127);  
		// setDriveMaxSpeed(110); 
	waitFinish(); 
	setDriveTimeout(450); 
	pwrIntake(0); 
}

void autonFar(bool flip){
	float x = getGyroGlobal(); 
	async::autonDrive(25, 100, 50);
	reachedError(.2); 
	FLY::set(2400);
	reachedError(.4); 
	pwrIntake(127);
	waitFinish();

	setDriveTimeout(150);
	// autonDrive(-34, 100, 50);
	resetGB(); 
	// async::autonDriveSlew(flip?-28:-29, /*{8, 11},*/ 100, 200, x); 
	// 	reachedError(.5); 
	// 	setDriveSlew(5); 
	// waitFinish(); 
	// setDriveSlew(0.4); 
	// setDriveTimeout(450);
	autonSweep(-35, {40, 4}, 100, 0);

	// X: 
	// autonDrive(4, 100, 0); 
	async::autonTurn(flip?81:-120, 80, 25);
		reachedError(.5); 
		pwrIntake(0); 
	waitFinish(); 
	// FLY::doubleShot(2400, 1790, 105, true);  
	// waitDouble();
	// delay(50);
	// async::autonSweep(flip?26:27, {flip?13:-12, flip?13:14}, 80, 200);
	x = getGyroGlobal(); 
	async::autonDrive(flip?24:13, 80, 0/*, x*/); 
		pwrIntake(0);
		// auto_S2=true; //S3
		reachedError(.95);
		DS::move(3000, 127);
	waitFinish();
	// autonTurn(15, 110, 0); 
	// DS::move(3000, 127); 
	delay(250); 
	pwrIntake(0); 
	resetGB();  //reset GYRO

	DS::move(400, 100); 

	// autonDrive(-8, 80, 0); 
	async::autonSweep(-11, {-15, 0}, {13, 5}, 60, 0); 
	waitFinish(); 
	pwrIntake(-127); 
	delay(260); 
	pwrIntake(0); 
	FLY::set(2100);
	delay(100); 
	// async::autonSweep(-23, {-15, 0}, {13, 10}, 90, 0); 
	autonDrive(-22, 100, 400);
		// reachedError(.9); 
	pwrIntake(-127); 
	delay(400); 
	pwrIntake(0); 
	delay(200); 
	autonSweep(-18, {flip?-85:85, 0}, 60, 100); 
	// autonTurn(89, 100, 0); 
	autonDrive(8, 60, 0);
	setDriveTimeout(9999);
	async::autonDriveNC(19, 108, 0); 
		reachedError(.2);
		pwrIntake(0); 
		reachedError(.4); 
		pwrIntake(127);  
		// setDriveMaxSpeed(110); 
	waitFinish(); 
	setDriveTimeout(450); 
	pwrIntake(0); 
}

void autonFN(bool flip){
	// FLY::set(2400); 
	// delay(800); 
	// goto X; 
	float x = getGyroGlobal(); 
	async::autonDrive(25, 100, 50);
	reachedError(.2); 
	FLY::set(2400);
	reachedError(.4); 
	pwrIntake(127);
	waitFinish();

	setDriveTimeout(150);
	// autonDrive(-34, 100, 50);
	resetGB(); 
	async::autonDriveSlew(flip?-28:-28, /*{8, 11},*/ 100, 200, x); 
		reachedError(.5); 
		setDriveSlew(5); 
	waitFinish(); 
	setDriveSlew(0.4); 
	setDriveTimeout(450);

	// X: 
	// autonDrive(4, 100, 0); 
	async::autonTurn(flip?81:-79, 80, 25);
		reachedError(.5); 
		pwrIntake(0); 
	waitFinish(); 
	// FLY::doubleShot(2400, 1790, 105, true);  
	// waitDouble();
	// delay(50);
	pwrIntake(-127);
	delay(260);
 	pwrIntake(0);
	delay(125);
	pwrIntake(127);
	delay(150);
	FLY::set(2400);
	// async::autonSweep(flip?26:27, {flip?13:-12, flip?13:14}, 80, 200);
	x = getGyroGlobal(); 
	async::autonSweep(flip?24:26, {flip?15:-13, flip?20:21}, 80, 0/*, x*/); 
		pwrIntake(0);
		// auto_S2=true; //S3
		reachedError(.28);
		pwrIntake(-127);
		reachedError(.95);
		DS::move(3000, 127);
	waitFinish();
	// autonTurn(15, 110, 0); 
	// DS::move(3000, 127); 
	delay(250); 
	pwrIntake(0); 
	FLY::set(2400);
	resetGB();  //reset GYRO
	// autonDriveNC(flip?-20:-21, 110, 0/*, x*/);
	autonSweep(flip?-12:-9, {flip?30:-25, flip?5:6}, 110, 0); 
	async::autonLock(flip?-85:90, flip?RIGHT:LEFT, 115, 0); 
		reachedError(.9); 
		DS::move(1600, 100); 
	waitFinish(); 
	// async::autonTurn(flip?-40:44, 100, 0); 
	// 	reachedError(.9); 
	// 	DS::move(1600, 100); 
	// waitFinish(); 
	// async::autonSweep(flip?-20:-13, {flip?19:-12, flip?1:5}, 100, 0); 	
	// 	reachedError(.4);
	// 	DS::move(1600, 100);
	// waitFinish(); 
	// autonTurn(flip?-74:76, 100, 0);

	async::autonDrive(flip?13:12, 80, 0);
		reachedError(.2); 
		setDriveMaxSpeed(40);  	
		reachedError(.8);
		DS::move(2450, 100);
	waitFinish(); 
	pwrIntake(127); 
	resetGB(); 
	autonDrive(-6, 70, 0); 
	DS::move(1800, 70); 
	delay(400);
	pwrIntake(0); 
	delay(50); 
	pwrIntake(127); 
	autonDrive(3, 80, 0);
	pwrIntake(127); 
	autonDrive(-3, 80, 0);  
	DS::move(3150, 80);
	pwrIntake(0);
	delay(100);
	FLY::doubleShot(2400, 1725, 105, false); //INTAKE NO STOP
	delay(100);
	async::autonDrive(18, 70, 0); 
		reachedError(.1); 
		DS::move(250, 110); 
	waitFinish(); 
	autonDrive(-6, 110, 0); 
}

void autonFCounter(bool flip){
	async::autonDrive(25, 100, 50);
	reachedError(.4); 
	pwrIntake(127);
	FLY::set(2300);
	waitFinish(); 

	setDriveTimeout(150);
	// autonDrive(-34, 100, 50);
	autonSweep(-28, {40, 4}, 100, 0);
	setDriveTimeout(450);
	pwrIntake(0); 
	autonTurn(-78, 100, 200);
	async::autonDrive(flip?9:11, 20, 0);
	reachedError(.1); 
	FLY::doubleShot(2400, 1725, 105, true); //INTAKE NO STOP
	reachedError(.25); 
	pwrIntake(0); 
	setDriveMaxSpeed(45); 
	reachedError(.8);
	DS::move(2600, 100);
	waitFinish(); 
	cntON = true; 
	pwrIntake(127); 
	autonDrive(-7, 70, 100); 
	DS::move(2000, 70);
	delay(1200);
	pwrIntake(0);
	delay(100);
	autonDrive(-9, 80, 0); 

	FLY::set(2500); 
	DS::move(700, 70);
	autonTurn(flip?43:-42, 60, 25);
	pwrIntake(-127);
	delay(250);
 	pwrIntake(0);
	delay(100);
	pwrIntake(127);  
	delay(150);
	// FLY::set(2100); 
	async::autonSweep(flip?26:25, {flip?10:-13, 14}, 80, 0);
	pwrIntake(0);
	// auto_S2=true; //S3
	reachedError(.18);
	pwrIntake(-127);
	reachedError(.8); 
	DS::move(1600, 100); 
	waitFinish();
}

/*
void autonSkills(){
	float x = getGyroGlobal();
	// goto xx; 

	DS::move(2400, 100); 
	async::autonDrive(25, 100, 50);
		reachedError(.4); 
		pwrIntake(127);
		reachedError(.9); 
		DS::move(1400, 90);   
	waitFinish(); 

	autonTurn(-30, 100, 0); 
	autonDrive(-14, 80, 0); 
	pwrIntake(0); 
	autonTurn(65, 100, 0); 
	async::autonDrive(12, 70, 0); 
		reachedError(.8); 
		DS::move(2700, 100); 
	waitFinish();
	DS::reset(); 
	async::autonSweep(-48, {58, 21}, 100, 100); 
		reachedError(.5); 
		DS::move(400, 100); 
		FLY::set(2500); 
	waitFinish(); 
	setDriveTimeout(150);
	autonSweep(-12, {-85, 0}, 80, 0); 
	setDriveTimeout(450);   

	
	// async::autonDrive(-7, 80, 0); 
	// FLY::set(2600); 
	// pwrIntake(127); 
	// setDriveTimeout(175);
	// async::autonSweep(-16, {-24, 9}, 60, 100);
	// 	reachedError(.4);
	// 	pwrIntake(0);
	// waitFinish();
	// DS::move(400, 100);
	// setDriveTimeout(450); 
	pwrIntake(127); 
	autonDrive(4, 60, 0); 
	pwrIntake(0);
	autonTurn(-81, 90, 0);

	pwrIntake(-127);
	delay(275);
 	pwrIntake(0);
	delay(125);
	pwrIntake(127);
	delay(150);
	FLY::set(2100);
	async::autonDrive(22, 70, 0);
		pwrIntake(0);
		// auto_S2=true; //S3
		reachedError(.3);
		pwrIntake(-127);
	waitFinish(); 
	FLY::set(2500);
	pwrIntake(-127); 
	autonTurn(-20, 100, 0); 
	setDriveTimeout(250); 
	autonSweep(17, {18, 1}, 90, 0); //slow for pickup
	setDriveTimeout(450); 
	 
 	// xx:
 	// pwrIntake(-127); //TEMPORARY
 	// FLY::set(2500); 

	async::autonSweep(-39, {-85, 10}, 100, 250);
		reachedError(.3); 
		pwrIntake(0); 
	waitFinish();
	delay(100);
	DS::move(2400, 100); 
	autonTurn(176, 100, 0);
	delay(100);

	pwrIntake(127);
	async::autonDrive(16, 60, 0);
		reachedError(.5); 
		DS::move(1100, 120);
	waitFinish(); 
	// DS::move(1100, 100); 
	autonSweep(-8, {14, 0}, 70, 100); 
	pwrIntake(0); 
	// autonDrive(-8, 90, 100); 
	// setDriveTimeout(150); 
	// autonSweep(-10, {-90, 0}, 70, 100); 
	// setDriveTimeout(450); 
	// autonDrive(4, 60, 0); 
	// autonTurnHP(22, 100, 0); 

	// pwrIntake(-127);
	// delay(240);
 // 	pwrIntake(0);
	// delay(125);
	// pwrIntake(127);  
	// delay(150);
	// FLY::set(2100);
	// async::autonDrive(20, 90, 0);
	// pwrIntake(0);
	// // auto_S2=true; //S3
	// reachedError(.15);
	// pwrIntake(-127);
	// waitFinish(); 
	// pwrIntake(0); 
	// FLY::set(2600);
	// autonDrive(-20, 90, 0);
	// async::autonDrive(-7, 70, 50);
	// reachedError(.2); 
	// pwrIntake(0);
	// waitFinish(); 

	//NEXT BALL TURN TO SHOOT MIDDLE FLAG
	autonTurn(-131, 90, 100);
	async::autonDrive(9, 55, 0);
		reachedError(.94);
		DS::move(3150, 127); //hold pid down after backup
	waitFinish();
	DS::reset(); 
	delay(50);
	pwrIntake(127); 
	async::autonDrive(-6, 40, 80);
		reachedError(.5);
		DS::move(1800, 100);
		setDriveMaxSpeed(60);
	waitFinish(); 
	delay(50); 
	DS::move(3150, 100); 
	delay(250);  
	async::autonDrive(15, 60, 0);
		reachedError(.4); 
		DS::move(400, 127); 
	waitFinish();
	delay(200); 
	// autonDrive(-8, 80, 0);
	pwrIntake(0);
	setDriveTimeout(150); 
	async::autonSweep(-29, {30, 8}, 70, 50);
		reachedError(.4); 
		pwrIntake(127); 
	waitFinish(); 
	setDriveTimeout(450);
	pwrIntake(0); 
	autonDrive(5, 60, 0); 
	autonTurn(19, 100, 0); 

	pwrIntake(-127);
	delay(275);
 	pwrIntake(0);
	delay(150);
	pwrIntake(127);  
	delay(150);
	pwrIntake(0);
	delay(50); 
	FLY::set(2100);
	setDriveTimeout(150); 
	async::autonSweep(38, {-23, 21}, 90, 100);
		reachedError(.06);
		pwrIntake(-127);
	waitFinish();
	setDriveTimeout(450);
	pwrIntake(0); 

	// xx:
	FLY::set(2600);

	//                       // MAJOR NEXT TURN
	autonSweep(-25, {-115, 0}, 100, 200);
	// DS::move(3150, 100);
	DS::move(1600, 90);
	autonTurn(-92, 90, 0); 
	async::autonDrive(11, 70, 0);
		reachedError(.15); 
		setDriveMaxSpeed(40); 
		reachedError(.9);
		DS::move(2250, 60);
	waitFinish(); 
	delay(50); 
	pwrIntake(127); 
	// DS::reset();
	async::autonDrive(-8, 70, 100); 
		reachedError(.7); 
		DS::move(1400, 70); 
	waitFinish(); 
	delay(1200);
	DS::move(3150, 80);
	delay(100);
	async::autonSweep(18, {-49, 3}, 60, 100);
		reachedError(.15); 
		DS::move(300, 100);
	waitFinish(); 
	pwrIntake(0);

	autonDrive(-5, 80, 150);
	autonTurn(-88, 90, 0);

	x = getGyroGlobal();
	// autonDrive(23, 100, 0, x);
	// pwrIntake(127); 
	// async::autonDrive(13, 70, 0, x);
	// reachedError(.7); 
	// DS::move(2400, 80); 
	// waitFinish();
	// async::autonDrive(-9, 70, 100); 
	// reachedError(.5); 
	// // DS::move(2000, 70); 
	// DS::move(3150, 80);
	// waitFinish(); 
	// delay(100);
	// async::autonDrive(20, 70, 0); 
	// reachedError(.4); 
	// DS::move(400, 110); 
	// waitFinish();
	// pwrIntake(0); 
	// autonDrive(12, 90, 100); 
	// async::autonDrive(8, 70, 0);
	// reachedError(.2); 
	// DS::move(400, 120); 
	// waitFinish(); 
	// autonDrive(-1, 80, 50); 

	// // autonDrive(-3, 80, 0, x);
	// autonTurn(-87, 110, 50);
	setDriveTimeout(150);
	autonDrive(20, 100, 50); //RESET WALL
	setDriveTimeout(450);

	// xx:
	// FLY::set(2600); 
	// delay(500); 
	autonSweep(-16, {11, 8}, 90, 0);
	// autonDrive(-15, 100, 0);
	// reachedError(.7);  
	pwrIntake(-127);
	delay(325);
 	pwrIntake(0);
 	delay(175);
 	pwrIntake(127); 
	async::autonDrive(-18, 100, 50);
		reachedError(.08); 
		pwrIntake(0); 
	waitFinish(); 
	pwrIntake(0); 
	delay(100); 
	// DS::move(2400, 100); 
	// autonTurn(5, 90, 0); //tiny turn
	pwrIntake(-127);
	delay(675); 
	pwrIntake(0);
	delay(175); 

	// autonDrive(-34, 100, 200); 
	// autonDrive(-3, 90, 0);
	// autonSweep(-5, {-10, 0}, 80, 0); 

	//turn to DS grab
	autonTurn(80, 110, 0);  //IF EXTRA TIME ADD TO SHOOT ANOTHER BALL FROM INTAKED BALL
	// async::autonDrive(-2, 90, 0); 
		// reachedError(.5); 
		// DS::move(2400, 100); 	
	waitFinish(); 
	pwrIntake(127); 
	auto_stop=true; 
	setDriveTimeout(150); 
	autonDrive(40, 110, 0); 
	setDriveTimeout(450); 
	autonSweep(-22, {-90, 1}, 100, 0);
	// autonDrive(15, 80, 100);
	// pwrIntake(0); 
	// autonDrive(-8, 80, 200);
	// pwrIntake(0); 
	// async::autonDrive(11, 60, 0); 
	// 	reachedError(.5); 
	// 	DS::move(2650, 100);
	// waitFinish(); 
	// DS::reset(); //DS=0

	
	autonTurn(-91, 100, 100);
	setDriveTimeout(150); 
	autonDrive(-10, 80, 100); 
	pwrIntake(127);
	autonDrive(8, 60, 0);
	setDriveTimeout(9999);
	async::autonDriveNC(30, 105, 0); 
		reachedError(.2);
		pwrIntake(0); 
		reachedError(.4); 
		pwrIntake(127);  
		// setDriveMaxSpeed(110); 
	waitFinish(); 
	setDriveTimeout(450); 
	pwrIntake(0); 
}*/


void autonSkills(){
	float x = getGyroGlobal(); 
	// goto xx; 
	
	DS::move(1700, 80); 
	autonDrive(7, 60, 0); 
	async::autonDrive(4, 80, 100, x); 
		reachedError(.55); 
		DS::move(2600, 80);
		reachedError(.8); 
		pwrIntake(127); 	 	
	waitFinish();
	async::autonDrive(-5, 60, 100, x); 
	FLY::set(2500); 
	// DS::move(2300, 100); 
	pwrIntake(127); 
	waitFinish(); 
	// async::autonDrive(-7, 80, 0); 
	// FLY::set(2600); 
	// pwrIntake(127); 
	setDriveTimeout(175); 
	async::autonSweep(-16, {-24, 9}, 60, 100);
		reachedError(.4); 
		pwrIntake(0); 
	waitFinish(); 
	DS::move(400, 100);
	setDriveTimeout(450); 
	pwrIntake(127); 
	autonDrive(4, 60, 0); 
	pwrIntake(0);
	autonTurn(-83, 90, 0); 

	pwrIntake(-127);
	delay(275);
 	pwrIntake(0);
	delay(125);
	pwrIntake(127);
	delay(150);
	FLY::set(2100);
	async::autonDrive(22, 70, 0);
		pwrIntake(0);
		// auto_S2=true; //S3
		reachedError(.3);
		pwrIntake(-127);
	waitFinish(); 
	FLY::set(2500);
	pwrIntake(-127); 
	autonTurn(-20, 100, 0); 
	setDriveTimeout(250); 
	autonSweep(17, {18, 1}, 90, 0); //slow for pickup
	setDriveTimeout(450); 
	 
 	// xx:
 	// pwrIntake(-127); //TEMPORARY
 	// FLY::set(2500); 

	async::autonSweep(-39, {-85, 10}, 100, 250);
		reachedError(.3); 
		pwrIntake(0); 
	waitFinish();
	delay(100);
	DS::move(2400, 100); 
	autonTurn(174, 100, 0);
	delay(100); 

	pwrIntake(127);
	async::autonDrive(16, 60, 0);
		reachedError(.5); 
		DS::move(1100, 120);
	waitFinish(); 
	// DS::move(1100, 100); 
	autonSweep(-8, {14, 0}, 70, 100); 
	pwrIntake(0); 
	// autonDrive(-8, 90, 100); 
	// setDriveTimeout(150); 
	// autonSweep(-10, {-90, 0}, 70, 100); 
	// setDriveTimeout(450); 
	// autonDrive(4, 60, 0); 
	// autonTurnHP(22, 100, 0); 

	// pwrIntake(-127);
	// delay(240);
 // 	pwrIntake(0);
	// delay(125);
	// pwrIntake(127);  
	// delay(150);
	// FLY::set(2100);
	// async::autonDrive(20, 90, 0);
	// pwrIntake(0);
	// // auto_S2=true; //S3
	// reachedError(.15);
	// pwrIntake(-127);
	// waitFinish(); 
	// pwrIntake(0); 
	// FLY::set(2600);
	// autonDrive(-20, 90, 0);
	// async::autonDrive(-7, 70, 50);
	// reachedError(.2); 
	// pwrIntake(0);
	// waitFinish(); 

	//NEXT BALL TURN TO SHOOT MIDDLE FLAG
	autonTurn(-133, 90, 100);
	async::autonDrive(10, 55, 0);
		reachedError(.94);
		DS::move(3150, 127); //hold pid down after backup
	waitFinish();
	DS::reset(); 
	delay(50);
	pwrIntake(127); 
	async::autonDrive(-6, 40, 80);
		reachedError(.5); 
		DS::move(1800, 100); 
		setDriveMaxSpeed(60); 
	waitFinish(); 
	delay(50); 
	pwrIntake(0); 
	DS::move(3150, 100); 
	delay(250);  
	pwrIntake(127); 
	async::autonDrive(14, 60, 0);
		reachedError(.4); 
		DS::move(400, 127); 
	waitFinish();
	delay(200); 
	// autonDrive(-8, 80, 0);
	pwrIntake(0);
	setDriveTimeout(150); 
	async::autonSweep(-29, {31, 9}, 70, 50);
		reachedError(.4); 
		pwrIntake(127); 
	waitFinish(); 

	setDriveTimeout(450);
	pwrIntake(0); 
	autonDrive(5, 60, 0); 
	autonTurn(19, 100, 0); 

	pwrIntake(-127);
	delay(275);
 	pwrIntake(0);
	delay(125);
	pwrIntake(127);  
	delay(110);
	pwrIntake(0);
	FLY::set(2100);
	setDriveTimeout(150); 
	async::autonSweep(38, {-23, 21}, 90, 100);
		reachedError(.06);
		pwrIntake(-127);
	waitFinish();
	setDriveTimeout(450);
	pwrIntake(0); 

	// xx:
	FLY::set(2600);

	//                       // MAJOR NEXT TURN
	autonSweep(-25, {-115, 0}, 100, 200);
	// DS::move(3150, 100);
	DS::move(1600, 90);
	autonTurn(-91, 90, 0); 
	async::autonDrive(13, 70, 0);
		reachedError(.3); 
		setDriveMaxSpeed(40); 
		reachedError(.9);
		DS::move(2475, 60);
	waitFinish(); 
	delay(50); 
	pwrIntake(127); 
	// DS::reset();
	async::autonDrive(-5, 60, 100); 
		reachedError(.7); 
		DS::move(1400, 70); 
	waitFinish(); 
	delay(800);
	DS::move(3150, 80);
	autonDrive(-2, 60, 0); 
	delay(100);
	async::autonSweep(17, {-47, 3}, 40, 100);
		reachedError(.13); 
		DS::move(300, 70);
		reachedError(.5);
		setDriveMaxSpeed(60);  
	waitFinish(); 
	pwrIntake(0);
	autonDrive(-5, 80, 150);
	autonTurn(-87, 90, 0);

	x = getGyroGlobal();
	// autonDrive(23, 100, 0, x);
	// pwrIntake(127); 
	// async::autonDrive(13, 70, 0, x);
	// reachedError(.7); 
	// DS::move(2400, 80); 
	// waitFinish();
	// async::autonDrive(-9, 70, 100); 
	// reachedError(.5); 
	// // DS::move(2000, 70); 
	// DS::move(3150, 80);
	// waitFinish(); 
	// delay(100);
	// async::autonDrive(20, 70, 0); 
	// reachedError(.4); 
	// DS::move(400, 110); 
	// waitFinish();
	// pwrIntake(0); 
	// autonDrive(12, 90, 100); 
	// async::autonDrive(8, 70, 0);
	// reachedError(.2); 
	// DS::move(400, 120); 
	// waitFinish(); 
	// autonDrive(-1, 80, 50); 

	// // autonDrive(-3, 80, 0, x);
	// autonTurn(-87, 110, 50);
	setDriveTimeout(150);
	autonDrive(20, 100, 50); //RESET WALL
	setDriveTimeout(450);
	autonSweep(-16, {11, 8}, 90, 0);
	// autonDrive(-15, 100, 0);
	// reachedError(.7);  
	pwrIntake(-127);
	delay(325);
 	pwrIntake(0);
 	delay(175);
 	pwrIntake(127); 
	async::autonDrive(-18, 100, 50);
		reachedError(.08); 
		pwrIntake(0); 
	waitFinish(); 
	pwrIntake(0); 
	delay(100); 
	DS::move(3050, 100); 
	// autonTurn(5, 90, 0); //tiny turn
	pwrIntake(-127);
	delay(675); 
	pwrIntake(0);
	delay(175); 

	// autonDrive(-34, 100, 200); 
	// autonDrive(-3, 90, 0);
	// autonSweep(-5, {-10, 0}, 80, 0); 

	//turn to DS grab
	autonTurn(-95, 90, 150);  //IF EXTRA TIME ADD TO SHOOT ANOTHER BALL FROM INTAKED BALL
	// async::autonDrive(-2, 90, 0); 
		// reachedError(.5); 
		// DS::move(2400, 100); 	
	waitFinish();
	pwrIntake(127); 
	auto_stop=true; 
	// autonDrive(15, 80, 100);
	async::autonDrive(10, 60, 0);
		reachedError(.2); 
		DS::move(400, 70);
	waitFinish();
	// pwrIntake(0); 
	// autonDrive(-8, 80, 200);
	// pwrIntake(0); 
	// async::autonDrive(11, 60, 0); 
	// 	reachedError(.5); 
	// 	DS::move(2650, 100);
	// waitFinish(); 
	// DS::reset(); //DS=0
	async::autonDrive(-26, 110, 50); 
		reachedError(.7); 
		DS::move(400, 80); 
	waitFinish(); 

	// async::autonDrive(14, 80, 150); 
	// reachedError(.1); 
	// DS::move(400, 80);
	// waitFinish(); 
	// DS::move(3800, 80); 

	// autonDrive(-17, 90, 150);

	// //TURN TO SHOOT FAR RIGHT
	// delay(200);
	// autonTurn(94, 80, 150);  //HP

	// // float x = getGyroGlobal(); //ADD
	// autonDrive(-2, 80, 0);
	// pwrIntake(0); 
	// delay(100); 
	// pwrIntake(-127);
	// delay(240);
 // 	pwrIntake(0);
	// delay(125);
	// pwrIntake(127);  
	// delay(150);

	// async::autonDrive(18, 127, 0);
	// reachedError(.6);
	// pwrIntake(-127);
	// waitFinish(); 
	// pwrIntake(0);

	// autonSweep(-18, {-90, 1}, 90, 25);
	// // autonDrive(-15, 90, 100); 
	// // autonTurn(92, 90, 50);
	// auto_stop=true; 
	// autonDrive(13, 110, 100);
	autonTurn(-81, 100, 100);
	autonDrive(19, 100, 100);
	autonTurn(93, 100, 100);
	setDriveTimeout(150); 
	autonDrive(-10, 80, 100); 
	pwrIntake(127);
	autonDrive(8, 60, 0);
	setDriveTimeout(9999);
	async::autonDriveNC(30, 108, 0); 
		reachedError(.2);
		pwrIntake(0); 
		reachedError(.4); 
		pwrIntake(127);  
		// setDriveMaxSpeed(110); 
	waitFinish(); 
	setDriveTimeout(450); 
	pwrIntake(0); 
}

void auton3(bool flip){
	float x = getGyroGlobal(); 
	//DS::init(0);  
	async::autonDrive(25, 100, 100, x); 
		reachedError(.4); 
		pwrIntake(127); 
		FLY::set(2150); 
	waitFinish(); 
	autonDrive(flip?-5:-4, 50, 150, x); 
	autonTurnHP(flip?58:-54, 70, 100);
	pwrIntake(0); 
	delay(4000);
	
	// delay(500); 
	pwrIntake(-127);
	delay(260);
	pwrIntake(0);
	delay(500); 
	pwrIntake(127); 
	delay(150); 
	pwrIntake(0);
	delay(350); 
	FLY::set(2000); 
	delay(750); 
	pwrIntake(-127); 
	delay(400); 
	pwrIntake(0); 

	delay(200);
	// autonTurn(30, 70, 100); 
	autonLock(flip?33:-33, LEFT, 100, 100); 
	auto_stop=true; 
	autonDriveNC(flip?20:14, 127, 0); 
}
