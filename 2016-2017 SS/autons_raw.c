#include "auton_library_97.c"
void autoMode0(){
	startTask(armControl);
	autonDrive(4, -127, false);  
	delay(100); 
	pwrClaw(-90); 
	delay(50); 
	pwrClaw(0); 
	delay(350);
	//putOutClawMid(); 
	auto_6D = true;
	delay(750);	 
	autonDrive(40, 120, false);
	delay(100);
	pwrClaw(127);
	delay(400);
	pwrClaw(75);	
	auto_8R = true;
	autonArc(38, -127, -28); 
	autonDrive(8, -120, false);
	auto_6U = true;
	autonDrive(16, -120);
	delay(400);

	auto_6D=true;
	autonDrive(10, -120);
	delay(650);

	pwrClaw(0); 
	/*autonDrive(4, 120);
	autonTurn(-75, 127);
	setArmMotor(-10); 
	delay(50);
	autonDrive(23, 127, false);
	pwrClaw(127);
	delay(250);
	auto_8R = true;
	delay(50); 
	autonTurn(80, 120);
	autonDrive(18, -120, false);
	auto_6U = true;
	autonDrive(7, -120, false);  
	pwrClaw(0);  
	delay(500); 

	auto_6D=true;
	delay(650);
	autonDrive(8, -120);
	delay(3000);
	autonDrive(26, 127);
	pwrClaw(127);
	delay(500);
	pwrClaw(100);
	delay(200); 
	//auto_8L=true; 
	autonDrive(16, -127); 
	auto_6U=true; 
	autonDrive(15, -127); 
	delay(500);
	auto_6D=true;
	delay(650);
	autonDrive(8, -120);
	pwrClaw(0);*/
}

void autoMode_prg(){
	startTask(armControl);
	autonDrive(4, -127, false);  
	delay(100); 
	pwrClaw(-90); 
	delay(50); 
	pwrClaw(0); 
	delay(350);
	//putOutClawMid(); 
	auto_6D = true;
	delay(750);	
	setArmMotor(0); 
	autonDrive(50, 120, false);
	delay(100);
	pwrClaw(127);
	delay(400);
	pwrClaw(110);	
	auto_8R = true;
	autonArc(38, -127, -28); 
	autonDrive(8, -120, false);
	auto_6U = true;
	autonDrive(16, -120);
	delay(400);

	auto_6D=true;
	delay(650);
	pwrClaw(0);
	autonDrive(8, -120, false);
	autonDrive(26, 127, false);
	delay(750); 
	pwrClaw(127);
	delay(500);
	pwrClaw(100);
	delay(200); 
	auto_8R=true; 
	autonDrive(16, -127); 
	auto_6U=true; 
	autonDrive(15, -127); 
	delay(500);
	
	auto_6D=true;
	delay(650);
	pwrClaw(0);
	autonDrive(8, -120, false);
	autonDrive(26, 127, false);
	delay(750); 
	pwrClaw(127);
	delay(500);
	pwrClaw(100);
	delay(200); 
	auto_8R=true; 
	autonDrive(16, -127); 
	auto_6U=true; 
	autonDrive(15, -127); 
	delay(500);

	auto_6D=true;
	delay(650);
	pwrClaw(0);
	autonDrive(8, -120, false);
	autonDrive(26, 127, false);
	delay(750); 
	pwrClaw(127);
	delay(500);
	pwrClaw(100);
	delay(200); 
	auto_8R=true; 
	autonDrive(16, -127); 
	auto_6U=true; 
	autonDrive(15, -127); 
	delay(500);

	auto_6D=true; 
	delay(650);
	pwrClaw(0);
	autonDrive(8, -120, false);
	autonDrive(9, 127); 
	delay(100); 
	autonTurn(-92, 127); 
	autonDrive(36, 127, false);
	pwrClaw(127);
	delay(500);
	pwrClaw(100);
	delay(50); 
	auto_8R=true; 
	delay(100); 
	autonTurn(90, 127); 
	delay(50); 
	autonDrive(10, -127, false); 
	auto_6U=true; 
	autonDrive(7, -127, false); 
	delay(850); 

	auto_6D=true; 
	delay(750);
	pwrClaw(0);
	autonDrive(7, 127);
	autonTurn(-65, 127);
	autonDrive(40, 127, false); 
	pwrClaw(127);
	delay(500);
	pwrClaw(100);
	delay(50); 
	auto_8R=true; 
	autonDrive(10, -127); 
	autonTurn(60, 127); 
	autonDrive(23, -127, false);
	auto_6U=true; 
	delay(650);
	autonDrive(10, -127, false); 

	autonTurn(-45, 127); 
	autonDrive(34, 127, false);
	pwrClaw(127);
	delay(500);
	pwrClaw(100);
	delay(50); 
	auto_8R=true; 
	delay(50); 
	autonDrive(10, -127); 
	autonTurn(20, 127); 
	autonDrive(20, -127, false); 
	auto_6U=true; 
	delay(700); 
	pwrClaw(0);
	
}

void autoMode1(){	
	startTask(armControl);
	autonDrive(2, -127);  
	auto_6D=true; 
	putOutClaw(); 
	delay(200);
	autonTurn(85, 127); 
	autonDrive(26, 127, false);
	pwrClaw(127);
	delay(200);
	pwrClaw(100);
	auto_8R = true;
	delay(75); 
	autonTurn(80, 127, false); 
	autonDrive(15, -127);

	auto_6U = true; 
	autonDrive(15, -127); 
	delay(600); 
	
	auto_6D=true;  
	delay(800); 
	autonDrive(30, 127, false);
	pwrClaw(127);
	delay(500);
	auto_8R=true;
	pwrClaw(100);
	delay(75); 
	//auto_8L=true; 
	autonDrive(40, -127); 
	auto_6U=true; 
	autonDrive(7, -127); 
	delay(600);
	auto_6D=true;  
	autonDrive(7, -127); 
	delay(650); 
    pwrClaw(0); 	
	autonDrive(37, 127, false);
	pwrClaw(127);
	delay(500);
	auto_8R=true;
	pwrClaw(100);
	delay(75); 
	//auto_8L=true; 
	autonDrive(40, -127); 
	auto_6U=true; 
	autonDrive(7, -127); 
	delay(600);
	auto_6D=true;  
	autonDrive(7, -127); 
	delay(850); 
	setArmMotor(0); 
	pwrClaw(0); 
}

/************************
	void pwrClaw(int pwr);
	void pwrDriveLeft(int speed);
	void pwrDriveRight(int speed);
	void pwrDrive(int speed);

	void autonDrive(int inches, int speed );
	void autonTurn(int degrees, int speed );

	void autonDrive(float distance, int speed, int correction = 5 );
	void autonTurn(int degrees, int speed );
	void autonArc(float distance, int speedL, int speedR, int correction = 5);

************************/
void auton_test(){
	startTask(armControl); 
	pwrDrive(100); 

}

