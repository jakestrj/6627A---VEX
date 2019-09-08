//distance=inches
//delay=msec
//autonDrive(distance, speed, delayAFTER)
//autonTurn(degrees, speed, delayAFTER)
//autonMG(OUT/IN)
//autonClaw(OUT/IN)
//delay(msec)

void skills178(){
	startTask(MobileGoalControl);
	int j = getGyroGlobal(); //DONT TOUCH
	pwrClaw(-30);
	autonMG(OUT);
	delay(100);
	autonDrive(50, 110, 200, j);
	autonMG(IN);
	delay(500);
	autonDrive(-48, 110, 200);
	//driving back to bar
	autonClaw(OUT);
	autonTurnLoad(45, 110, 200);
	autonDrive(-29, 110, 200);
	autonTurnLoad(92, 110, 200);
	autonDrive(21, 90, 0);
	autonMG(OUT);
	autonDrive(4, 90, 50);
	delay(250);
	setMGLiftMotor(0);
	autonMG(IN);
	delay(100);

	//backing out
	autonDrive(-20, 100, 100);
	autonTurn(-90, 100, 100);
	autonDrive(16, 100, 200);
	autonMG(OUT);
	autonTurnLoad(-88, 110, 100);
	delay(200);
	int k = getGyroGlobal(); //DONT TOUCH
	autonDrive(30, 100, 300/*, k*/);
	autonMG(IN);
	delay(200);
	
	autonDrive(3, 60, 0);
	delay(100);
	autonDrive(-36, 100, 200);
	autonTurnLoad(188, 110, 200);
	autonMG(OUT);
	autonDrive(12, 90, 0);

	//place second mgoal
	int o = getGyroGlobal(); //DONT TOUCH
	delay(200);
	autonMG(IN);
	//go back for third
	autonDrive(-43, 100, 250, o); 
	autonMG(OUT);
	autonTurn(98, 110, 100);
	autonDrive(26, 110, 0);
	autonDrive(11, 110, 0);
	autonMG(IN);
	delay(400);
	//just after third grab

	autonTurn(-98, 100, 50);
	autonDrive(43, 100, 100);
	autonMG(OUT);
	autonDrive(5, 80, 100);
	delay(200);
	int i = getGyroGlobal(); //DONT TOUCH
	autonMG(IN);

	autonDrive(-15, 80, 200, i); //maintain heading
	autonMG(IN);
	delay(100);
	//turn for LONG run
	autonTurn(190, 90, 150);
	autonMG(OUT);
	autonDrive(8, 80, 0);
	autonDrive(67, 110, 50);
	autonMG(IN);
	delay(500);
	
	autonDrive(30, 110, 100);
	autonTurn(90, 90, 100);
	autonDrive(19, 90, 100);
	autonTurn(-90, 90, 100);

	//place fourth mgoal
	autonDrive(23, 90, 0);
	autonMG(OUT);
	autonDrive(4, 90, 50);
	delay(200);
	setMGLiftMotor(0);
	//20 point zone
	autonMG(IN);
	delay(100);
	autonDrive(-21, 100, 100);

	autonTurn(90, 100, 100);
	autonDrive(17, 100, 100);
	autonMG(OUT);
	autonTurn(93, 100, 100);
	delay(200);
	//go for fifth
	autonDrive(31, 100, 100);
	autonMG(IN);
	delay(200);
	autonDrive(4, 90, 100);
	autonTurn(180, 100, 100);
	
	autonDrive(41, 100, 100);
	autonMG(OUT);
	autonDrive(5, 80, 100);
	delay(200);
	autonMG(IN);
	autonDrive(-10, 100, 100);
	autonTurn(125, 100, 100);
	autonMG(OUT);
	
	//go for sixth
	autonDrive(53, 110, 100);
	autonMG(IN);
	delay(200);
	autonDrive(4, 90, 100);
	autonDrive(-55, 120, 100);
	autonTurn(-25, 120, 0);
	autonDrive(-15, 100, 0);
	autonTurn(-100, 110, 100);
	autonMG(OUT);
	autonDrive(9, 100, 100);
	autonMG(IN);

	//back out
	autonDrive(-8, 110, 0);
	//fast for parking
	autonTurnFast(50, 127, 0);
	autonDriveFast(-80, 127, 0);

	pwrDriveRight(0);pwrDriveLeft(0); setMGLiftMotor(0);
}
