#include "main.h"
#include <stdio.h>
#include <stdbool.h>
using namespace pros::c;
using namespace pros;

Timer auton_timer; 
void autonomous() {
  DISABLED = false; 
  auton_timer.reset(); 
  reset_task_control();
  resetGB();

  switch (autonomousMode){
    case 0: autonFP(false); 
    break;
	  case 1: autonFP(true);
    break;
	  case 2: autonSkills(); 
    break;
	  case 3:
    break;
	  case 4: 
    break;
	  default: break;
  }

  DISABLED = true; 
}
