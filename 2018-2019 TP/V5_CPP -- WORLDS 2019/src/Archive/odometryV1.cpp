void odoV1(void * param){ // TASK
	// GEARING
	int prevError = 0;

	while(true){
			long start = millis();

			double deltaL = (encdVal.left - prevEncdL)*inPerDeg;
			double deltaR = (encdVal.right - prevEncdR)*inPerDeg;
			double deltaAngle = (deltaL-deltaR)/baseWidth;		//theta = (s2-s1)/width
			double avgEncdsChange = (deltaL+deltaR)/2;

			if(deltaAngle==0){
				abs_position.x += avgEncdChange*sin(abs_position.angle);           //Simple trig
				abs_position.y += avgEncdChange*cos(abs_position.angle);
			}
			else{
				double halfDeltaAngle = deltaAngle/2;
				double strDist = 2*(avgEncdChange/deltaAngle)*sin(halfDeltaAngle);
				abs_position.x += strDist * sin(abs_position.angle+(halfDeltaAngle));
				abs_position.y += strDist * cos(abs_position.angle+(halfDeltaAngle));
				abs_position.angle += deltaAngle;
			}

			prevEncdL = encdVal.left;
			prevEncdR = encdVal.right;

			printf("abs_position.x:%f",abs_position.x);
			printf("abs_position.y:%f",abs_position.y);
			printf("abs_position.angle:%f",abs_position.angle);

			error.x = target.x-abs_position.x;
			error.y = target.y-abs_position.y;

			//printf("origTargAngle:%f",origTargAngle);
			bool finalTurning = false;
			bool forceTurn = false;
			if(fabs(error.y) < 2){
				forceTurn = true;
				if(error.x >=2) target.angle = PI/2;
				else if(error.x <= -2) target.angle = -PI/2;
				else if(origTargAngle < 100) target.angle = origTargAngle;
				else {target.angle = abs_position.angle; finalTurning = true;}
			}
			else if(fabs(error.x) < 2){
				forceTurn = true;
				if(error.y >=2) target.angle = 0;
				else if(error.x <= -2) target.angle = PI;
				else if(origTargAngle < 100) target.angle = origTargAngle;
				else {target.angle = abs_position.angle; finalTurning = true;}
			}
			else if(fabs(error.x)>=2 || fabs(error.y)>=2) target.angle = atan(error.x/error.y);
			else if(origTargAngle < 100) target.angle = origTargAngle;
			else {target.angle = abs_position.angle; finalTurning = true;}


			error.angle = target.angle-abs_position.angle;
			//if(fabs(sqrt(error.x*error.x + error.y*error.y))>2 && error.x > 2) error.angle=atan(error.x/error.y);
			//printf("error.angle(sub):%f",error.angle);

			if(error.angle > PI) error.angle -= (2 * PI);
			else if(error.angle < -PI) error.angle += (2 * PI);

			if(!finalTurning){
				//printf("init error.angle:%f",error.angle);
				if(error.angle > PI/2) error.angle-= PI;
				else if(error.angle < -PI/2) error.angle += PI;
			}

			if(error.angle > PI) error.angle -= (2 * PI);
			else if(error.angle < -PI) error.angle += (2 * PI);

			bool reverseDir = false;
			//printf("fake target.angle:%f",error.angle+abs_position.angle);
			if(((error.x<0 && error.angle+abs_position.angle>=0 
				&& error.angle+abs_position.angle<=PI) 
				|| (error.x>0 && (error.angle+abs_position.angle<=0 
					|| error.angle+abs_position.angle>=PI)))&&fabs(error.x)>=2) 
				{reverseDir = true;printf("REV");}

			if(((error.y>0 && error.angle+abs_position.angle>=PI/2 
				&& error.angle+abs_position.angle<=3*PI/2) 
				|| (error.y<0 && (error.angle+abs_position.angle<=PI/2 
					|| error.angle+abs_position.angle>=3*PI/2)))&&fabs(error.y)>=2) 
				{reverseDir = true;printf("REV");}

			printf("error.x:%f",error.x);
			printf("error.y:%f",error.y);
			printf("error.angle:%f",error.angle);

			if(fabs(error.x) >= 2 || fabs(error.y) >= 2 || fabs(error.angle) >= ANGLE_LEEWAY){
				baseFinish = false;

				double lSpd,rSpd = 0;
				double err = sqrt(error.x*error.x + error.y*error.y);
				if(reverseDir) err*= -1;

				if(firstMove) {                           //Change prev error to current error if 1st time
					prevError = err;
					firstMove = false;
				}

				double sampleErrorAngle = error.angle>0?error.angle:-error.angle;
				if (sampleErrorAngle>PI) sampleErrorAngle -= 2*PI;
				else if(sampleErrorAngle<-PI) sampleErrorAngle += 2*PI;
				double correction;

				if((fabs(error.x) >= 2 || fabs(error.y) >= 2) && fabs(error.angle) <= MAX_SWERVE_ANGLE && (!forceTurn||fabs(error.angle)<ANGLE_LEEWAY)){
					if(finalMove){
		#ifdef DERIVATIVE_ENABLED
						deltaErr = prevError - err;       //PD Loop
						prevError = err;
						printf("deltaErr: %f",deltaErr);
						lSpd = kP*err+kD*deltaErr;
		#else
						lSpd = kP*err;                  //P Loop
		#endif
						rSpd = lSpd;
					}
					else{
						if(err>0) lSpd = 0.8 * baseRPM /60 * inPerRev; //Run at 80% of rpm
						else lSpd = -0.8 * baseRPM /60 * inPerRev;

						rSpd = lSpd;

					}
					correction = error.angle*baseWidth/2*kCorrectionTurn;
				}
				else{
					lSpd = 0;
					rSpd = 0;
					correction = error.angle*baseWidth/2*kPureTurn;
				}

				//Angle Corrections using equation s2-s1 = width*theta
				if(sampleErrorAngle >= ANGLE_LEEWAY){
					lSpd += correction / 2;
					rSpd -= correction / 2;
				}

				printf("right:%f",rSpd);

				if(fabs(lSpd) > baseRPM*inPerRev/60 || fabs(rSpd) > baseRPM*inPerRev/60){
					printf("reached limit");
					double ratio;
					if(fabs(lSpd)>fabs(rSpd)){
						bool negate = false;
						if(lSpd<0) negate = true;
						ratio = rSpd/lSpd;
						lSpd = baseRPM*inPerRev/60;
						if(negate) lSpd *= -1;
						rSpd = lSpd*ratio;
					} else{
						bool negate = false;
						if(rSpd<0) negate = true;
						ratio = lSpd/rSpd;
						rSpd = baseRPM*inPerRev/60;
						if(negate) rSpd *= -1;
						lSpd = rSpd*ratio;
					}
				}

				lSpd /= inPerRev;
				rSpd /= inPerRev;
				lSpd *= 60;
				rSpd *= 60;
				setWheelSpeed(lSpd,rSpd);
			}
			else{
				printf("--- RETURN true");
				baseFinish = true;
				setWheelSpeed(0,0);
			}
			delay(30);
		}
}