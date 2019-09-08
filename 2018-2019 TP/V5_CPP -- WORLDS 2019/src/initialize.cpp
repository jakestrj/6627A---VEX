#include "main.h"
#include <stdio.h>
#include <stdbool.h>
using namespace pros::c;
using namespace pros;

int lcdTelemMode;
bool mAutonBool;
int autonomousMode;
int autonNumber;
int global_distance; 

void LcdAutonomousSet( int value, bool select){
    if(select) autonomousMode = value;
    if(autonomousMode == value) mAutonBool = true;
    else mAutonBool = false;
}

void on_left_button(){
  if(--autonNumber < 0) autonNumber = MAX_CHOICE;
  LcdAutonomousSet(autonNumber, false);
}
void on_right_button(){
  if(++autonNumber > MAX_CHOICE) autonNumber = 0;
  LcdAutonomousSet(autonNumber, false);
}
void on_center_button(){
  LcdAutonomousSet( autonNumber, true );
}

void displayInfoTask(void *param){
	lcd::initialize();
	lcd::register_btn0_cb(on_left_button);
	lcd::register_btn1_cb(on_center_button);
	lcd::register_btn2_cb(on_right_button);

  long autonTime=0; 
  while(true){
    if(competition_is_autonomous() && !DISABLED) autonTime = auton_timer.getTime(); 

  	char str0[100], str1[100], str2[100], str3[100], str4[100], str_disabled[100];
    sprintf(str0, "RPM: %d", getVelF());
	  sprintf(str1, "%1.2f%% POT:%d", battery_get_capacity(), (int)getPotA());
  	sprintf(str2, "B(%d)%d", mAutonBool, autonNumber);
    sprintf(str_disabled, "AN: %d", autonomousMode);
    sprintf(str3, "IM:%d IL:%d IR:%d IA:%d", (int)getDM(), (int)getDL(), (int)getDR(), (int)reverseGyroCurve(getGyroGlobal()));
    sprintf(str4, "TIME: %d", (int)autonTime);
    lcd_set_text(0, str0);
    lcd_set_text(1, str1);
    lcd_set_text(2, competition_is_disabled()?str_disabled:str2);
    lcd_set_text(3, str3);
    lcd_set_text(4, str4);
    delay(25);
  }
}

adi_encoder_t sENCF, sENCL, sENCR; 
adi_gyro_t sGYRO;
void sensorInit(){
  sENCF = adi_encoder_init(pENCF_1, pENCF_2, false);
  sENCL = adi_encoder_init(pENCL_1, pENCL_2, true);
  sENCR = adi_encoder_init(pENCR_1, pENCR_2, true);
  sGYRO = adi_gyro_init(pGYRO, 1); 
  adi_port_set_config(pPOTA, E_ADI_ANALOG_IN);
  adi_port_set_config(pPOTC, E_ADI_ANALOG_IN);

  //initialize for vel pid
  motor_set_gearing(mDRIVEL_1, E_MOTOR_GEARSET_18); 
  motor_set_gearing(mDRIVEL_2, E_MOTOR_GEARSET_18); 
  motor_set_gearing(mDRIVER_1, E_MOTOR_GEARSET_18); 
  motor_set_gearing(mDRIVER_2, E_MOTOR_GEARSET_18); 
  motor_set_gearing(mFLYU, E_MOTOR_GEARSET_06); 
  motor_set_gearing(mFLYD, E_MOTOR_GEARSET_06); 
  motor_set_reversed(mDRIVEL_1, false);
  motor_set_reversed(mDRIVEL_2, false);
  motor_set_reversed(mDRIVER_1, false);
  motor_set_reversed(mDRIVER_2, false);
  motor_set_reversed(mFLYU, false);
  motor_set_reversed(mFLYD, false);
  motor_set_encoder_units(mDRIVEL_1, E_MOTOR_ENCODER_DEGREES); 
  motor_set_encoder_units(mDRIVEL_2, E_MOTOR_ENCODER_DEGREES); 
  motor_set_encoder_units(mDRIVER_1, E_MOTOR_ENCODER_DEGREES); 
  motor_set_encoder_units(mDRIVER_2, E_MOTOR_ENCODER_DEGREES);
  motor_set_encoder_units(mFLYU, E_MOTOR_ENCODER_DEGREES);
  motor_set_encoder_units(mFLYD, E_MOTOR_ENCODER_DEGREES);

}

void reset_task_control(){
   auto_stop = auto_S1 = auto_S2 = auto_S3 = auto_S4 = auto_S3SLOW = auto_drive = auto_turn = auto_sweep = auto_sweep2 = auto_lock = auto_pos = false;  
   DS::reset(); 
}

bool DISABLED = true; 
void initialize() {
	lcdTelemMode = 0; mAutonBool = true; autonNumber = 0; autonomousMode = 0;
  doubleShot = false; 
  
  //ONE TIME INITIALIZATION
  sensorInit();

  //SENSOR RESET ONLY
	reset_task_control();
  resetGB();
  resetDistance(); 

	task_t _displayInfoTask = TASK(displayInfoTask);
  task_t _fwVelTask = TASK(fwVelTask);
	task_t _flyControlTask = TASK(flyControlTask);
  task_t _dsControlTask = TASK(dsControlTask);
  task_t _autonATask = TASK(autonATask);
}

void disabled() {
  DISABLED = true; 
  motor_set_brake_mode(mDS, E_MOTOR_BRAKE_COAST); 
  motor_set_brake_mode(mLIFT, E_MOTOR_BRAKE_COAST); 
}

void competition_initialize(){}
