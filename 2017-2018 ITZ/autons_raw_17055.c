void auton1_2(bool left){
	global_STACKHEIGHT=0;
	setClaw(30);
	setTLiftMotor(-11); 
	autonLift(1250);
	delay(250);
	autonMG(OUT);
	autonDrive(72, 127, 100);
	autonMG(IN);
	delay(1100);
	autonStack();
	autonDrive(7, 127, 0);
	waitForAutonStack();
	delay(200);
	autonStack(true, true); //no reset
	delay(200);
	autonDrive(-55, 127, 50);
	autonLift(1300);
	setClaw(0); 
	autonArm(IN);
	delay(100); 
	if(left) autonArc(-130, 127, -6, 0);
	else autonArc(130, 6, -127, 0); 
	autonLift(1600); //1340
	autonDriveM(60, 127, 100);
	autonMG(IN);
	delay(150);
	autonDriveTask(-22, 127, 0); 
	delay(100); autonLift(1340); 
}

void auton1_3(bool left){
	global_STACKHEIGHT=0;
	setClaw(30);
	setTLiftMotor(-11); 
	autonLift(1250);
	delay(250);
	autonMG(OUT);
	autonDrive(72, 127, 100);
	autonMG(IN);
	delay(1100);
	autonStack();
	autonDrive(7, 127, 0);
	waitForAutonStack();
	delay(200);
	autonStack();
	delay(700);	
	int s = getGyroGlobal(); 
	autonDrive(9, 100, 0, s);
	waitForAutonStack();
	delay(200);
	autonStack(true, true); //no reset
	delay(200);
	autonDrive(-65, 127, 50, s);
	autonLift(1300);
	setClaw(0); 
	autonArm(IN);
	delay(100); 
	if(left) autonArc(-140, 127, -6, 0);
	else autonArc(130, 6, -127, 0); 
	autonLift(1600); //1340
	autonDriveM(60, 127, 100);
	autonMG(IN);
	delay(150);
	autonDriveTask(-22, 127, 0); 
	delay(100); autonLift(1340); 
}

void auton1_4(bool left){
	global_STACKHEIGHT=0;
	setClaw(30);
	setTLiftMotor(-15); 
	autonLift(1250);
	delay(250);
	autonMG(OUT);
	autonDrive(72, 127, 100);
	autonMG(IN);
	delay(400); 
	autonDriveTask(6, 127, 0); 
	delay(700);
	autonStack();
	waitForAutonStack();
	delay(200);
	autonStack();
	delay(700);
	int s = getGyroGlobal(); 
	autonDriveTask(6, 100, 0);
	waitForAutonStack();
	delay(200);

	autonStack();
	delay(700);
	autonDriveTask(9, 100, 0);
	waitForAutonStack();

	autonStack(true); //no reset
	delay(200);
	autonDrive(-68, 127, 0, s);
	autonLift(1300);
	setClaw(0); 
	autonArm(IN); 

	//if(left) autonArc(-140, 127, -6, 0); //140
	//else autonArc(140, 6, -127, 0); 

	if(left) autonArc(-128, 127, -12, 0); 
	else autonArc(128, 12, -127, 0); 

	setTLiftMotor(-15);
	autonLift(1600); //1340
	autonDriveM(54, 127, 100);
	autonMG(IN);
	delay(150);
	autonDriveTask(-22, 127, 0); 
	delay(100); autonLift(1340); 
}

void auton1_4_in5(bool left){
	global_STACKHEIGHT=0;
	setClaw(30);
	setTLiftMotor(-15); 
	autonLift(1250);
	delay(250);
	autonMG(OUT);
	int v = getGyroGlobal(); 
	autonDrive(72, 127, 100, v);
	autonMG(IN);
	delay(400); 
	autonDriveTask(7, 127, 0); 
	delay(700);
	autonStack();
	//autonDrive(6, 127, 0);
	waitForAutonStack();
	delay(200);
	autonStack();
	delay(700);
	int s = getGyroGlobal(); 
	autonDriveTask(8, 100, 0);
	waitForAutonStack();
	delay(200);

	autonStack();
	delay(700);
	autonDriveTask(7, 100, 0);
	waitForAutonStack();

	autonStack(true, true); //no reset
	delay(200);
	autonDrive(-63, 127, 0, s);
	autonLift(1300);
	setClaw(0); 
	autonArm(IN); 
	autonTurnLoad(left?190:-190, 127, 0); 
	autonMG(OUT);
	delay(700);
	autonDrive(-30, 127, 0); 
}

void auton1_3_in5(bool left){
	global_STACKHEIGHT=0;
	setClaw(30);
	setTLiftMotor(-15); 
	autonLift(1250);
	delay(250);
	autonMG(OUT);
	int v = getGyroGlobal(); 
	autonDrive(72, 127, 100, v);
	autonMG(IN);
	delay(400); 
	autonDriveTask(7, 127, 0); 
	delay(700);
	autonStack();
	//autonDrive(6, 127, 0);
	waitForAutonStack();
	delay(200);
	autonStack();
	delay(700);
	int s = getGyroGlobal(); 
	autonDriveTask(8, 100, 0);
	waitForAutonStack();
	delay(200);

	/*autonStack();
	delay(700);
	autonDriveTask(7, 100, 0);
	waitForAutonStack();*/

	autonStack(true, true); //no reset
	delay(200);
	autonDrive(-55, 127, 0, s);
	autonLift(1300);
	setClaw(0); 
	autonArm(IN); 
	autonTurnLoad(left?185:-185, 127, 0); 
	autonMG(OUT);
	delay(700);
	autonDrive(-30, 127, 0); 
}

void autonStat(){
	global_STACKHEIGHT=0;
	setClaw(30); 
	autonLift(1600);
	autonDrive(20, 127, 100);
	autonArm(OUT);
	delay(300);
	autonClaw(OUT); 
	delay(100); 
	autonLift(1600);
	autonArm(IN); 
	delay(200); 
	autonLift(1100); 
	autonDrive(-5, 127, 200);
	autonMG(OUT); 
	autonTurnLoad(-85, 127, 200); 
	autonDrive(28, 127, 0); 
	autonArm(OUT); 
	delay(300); 
	autonLift(_tick_baseHeight-100); 
	delay(500); 
	autonClaw(IN); 
	delay(100); 
	autonLift(1250); 
	delay(200); 
	autonArm(IN);
	autonTurn(-22, 127, 100);
	int s = getGyroGlobal(); 
	autonDrive(20, 127, 0); 
	autonMG(IN); 
	delay(400); 
	autonTurnTask(25, 127, 0); 
	delay(300); 
	/*autonDrive(8, 127, 0); 
	delay(150);
	autonStack(); 
	waitForAutonStack();*/
	autonStack(true); 
	delay(200); 
	autonDrive(-38, 127, 0); 
	autonLift(1775);
	autonArm(IN);
	setClaw(0); 
	autonMG(OUT); 
	autonTurnLoad(185, 127, 0); 
	autonDrive(-30, 127, 0); 
}

void autonLoad(bool left){
	global_STACKHEIGHT=0;
	setClaw(30);
	autonLift(1250);
	setTLiftMotor(-11);
	delay(250);
	autonMG(OUT);
	autonDrive(72, 127, 100);
	autonMG(IN);
	delay(1100);
	autonStack(true, true);
	autonDrive(-24, 127, 0);
	autonLift(1450);
	autonArm(OUT); 
	if(left) autonTurn(-47, 127, 100);
	else autonTurn(47, 127, 100);
	//autonDriveTask(-4, 127, 0); 
	autonLift(_tick_baseHeight-100);
	delay(300);  
	autonStackLoad();
	waitForAutonStackLoad(); 
	autonStackLoad();
	waitForAutonStackLoad(); 
	autonStackLoad(true); 
	delay(500); 

	autonTurn(-50, 127, 0); 
	autonLift(1340); autonArm(IN); 
	autonDrive(-26, 127, 50);
	if(left) autonArc(-125, 127, -12, 50); 
	else autonArc(125, 12, -127, 50); 
	autonLift(1800); //1340
	autonDriveM(66, 127, 100);
	autonMG(IN);
	delay(150);
	autonDriveTask(-22, 127, 0); 
	delay(100); autonLift(1340); 
}

void autonLoad5(){
	global_STACKHEIGHT=0;
	setClaw(30);
	autonLift(1250);
	delay(250);
	autonMG(OUT);
	autonDrive(72, 127, 100);
	autonMG(IN);
	delay(1100);
	autonStack(true, true);
	autonDrive(-29, 127, 0);
	autonLift(1300);
	autonArm(OUT); 
	autonTurn(47, 127, 100);
	autonDriveTask(-3, 127, 0); 
	autonStackLoad();
	waitForAutonStackLoad(); 
	autonStackLoad();
	waitForAutonStackLoad(); 
	autonStackLoad();
	waitForAutonStackLoad(); 
	autonStackLoad();
	waitForAutonStackLoad(); 
	autonStackLoad();
	waitForAutonStackLoad(); 
	autonStackLoad(true); 
	delay(500); 

	autonTurn(-50, 127, 0); 
	autonArm(IN); 
	autonLift(1340);
	autonDrive(-15, 127, 50);
	autonTurn(-180, 127, 0); 
	autonLift(1800); //1340
	autonDriveM(10, 127, 150);
	autonDrive(-20, 127, 0); 
}
