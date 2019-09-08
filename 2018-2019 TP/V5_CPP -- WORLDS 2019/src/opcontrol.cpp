#include "main.h"
#include <stdio.h>
#include <stdbool.h>
using namespace pros::c;
using namespace pros;

#define mainJoyY controller_get_analog(CONTROLLER_MASTER, ANALOG_RIGHT_Y) //ch2
#define mainJoyX controller_get_analog(CONTROLLER_MASTER, ANALOG_LEFT_X) //ch4
#define KEY(a) controller_get_digital(CONTROLLER_MASTER, a)
float MULT;
void logDrive(int l, int r){
	if(MULT==1){
		pwrDriveLeft((abs(l)*l/127));
		pwrDriveRight((abs(r)*r/127));
	}
	else{
		pwrDriveLeft(l*MULT);
		pwrDriveRight(r*MULT);
	}
}

void opcontrol() {
	DISABLED = true; 
	int threshold = 12;
	int mainX = 0,mainY = 0,secondaryX = 0,secondaryY = 0;
	MULT=1;
  	resetGB();
	reset_task_control();
	motor_set_brake_mode(mDRIVER_1, E_MOTOR_BRAKE_COAST); 
	motor_set_brake_mode(mDRIVER_2, E_MOTOR_BRAKE_COAST); 
	motor_set_brake_mode(mDRIVEL_1, E_MOTOR_BRAKE_COAST); 
	motor_set_brake_mode(mDRIVEL_2, E_MOTOR_BRAKE_COAST); 

	motor_set_brake_mode(mDS, E_MOTOR_BRAKE_HOLD); 
	motor_set_brake_mode(mLIFT, E_MOTOR_BRAKE_HOLD); 

	bool selectedAim = false; 
	int aim = 0; 

	while (true) {
		if(abs(mainJoyY) >= threshold)	mainY=mainJoyY;
		else	mainY = 0;
		if(abs(mainJoyX) >= threshold)	mainX=mainJoyX;
		else	mainX = 0;
		if(abs(mainY) < abs(secondaryY)) mainY=secondaryY;
		if(abs(mainX) < 40 && MULT==1) mainX /= 5;
		if(abs(mainY) < 40 && MULT==1) mainY /= 5;
		logDrive((mainY - mainX), (mainX + mainY));
		// chezyDrive(mainY, -mainX, false); 

		if(KEY(DIGITAL_R2)){ MULT=/*0.35*/0.3; threshold=0; }
		else if(!KEY(DIGITAL_R2)){ MULT=1; threshold=12; }

		if(!doubleShot && !KEY(DIGITAL_A)){
		    motor_move(mINTAKE, KEY(DIGITAL_L1) ? 127 :
		    (KEY(DIGITAL_L2) ? -127 : 0));
		}

		pwrDS( KEY(DIGITAL_UP) ? 80 :
		    (KEY(DIGITAL_DOWN) ? -80 : 0) );

		if(KEY(DIGITAL_LEFT)) pwrLift(127), selectedAim=false; 
		else if(KEY(DIGITAL_RIGHT)) pwrLift(-127), selectedAim=false; 
		else{
			pwrLift(0); 
			if(!selectedAim){ 
				aim = getArmC();
				selectedAim = true;
				setTarget(lift, aim); 
				cout << aim << endl; 
			}
			double pwr = -limit(lift.update(), 127); 
			pwrLift(pwr); 
		}

		/*if(KEY(DIGITAL_R1) && !pressed){ 
			reset_task_control();
  			resetGB();
			autonSkills(); 
			pressed=true;
		}*/
			

		delay(20);
	}
}
