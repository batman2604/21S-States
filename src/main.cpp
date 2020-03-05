#include "main.h"
//Motor Set Up
pros::Controller Master(pros::E_CONTROLLER_MASTER);
pros::Motor front_right(12, pros::E_MOTOR_GEARSET_18);
pros::Motor back_right(11, pros::E_MOTOR_GEARSET_18);
pros::Motor front_left(19, pros::E_MOTOR_GEARSET_18, true);
pros::Motor back_left(18, pros::E_MOTOR_GEARSET_18, true);
pros::Motor left_intake(15, pros::E_MOTOR_GEARSET_18);
pros::Motor right_intake(2, pros::E_MOTOR_GEARSET_18,true);
pros::Motor tray(13);
pros::Motor arm(3, pros::E_MOTOR_GEARSET_18);
//Global Variables
bool check;
int tray_pos;
int fast_val = 3300;
int fast_val2 = 3900;
int slow_val = 4500;
int slow_val2 = 4750;
int final_val = 4900;
//Start of Drive PID Set Up
int errorA;//500
int PrevErrorA;//625
int enValA;
int derivitiveA;
int integralA;
void F_Left(int target, float kP, float kI, float kD, float thresh){
    if(abs(errorA) > thresh){
      integralA = 0;
    }
    else {
    integralA += errorA;
  }
  if(abs(errorA) < 30){
    front_left.move(20);
  }
derivitiveA = errorA - PrevErrorA;
PrevErrorA = errorA;
enValA = front_left.get_position();
errorA = target - enValA;
front_left.move((errorA*kP) + (integralA * kI) + (derivitiveA * kD));
pros::delay(10);
}


int errorB;
int PrevErrorB;
int enValB;
int derivitiveB;
int integralB;
void B_Left(int target, float kP, float kI, float kD, float thresh ){
    if(abs(errorB) > thresh){
      integralB = 0;
    }
    else {
    integralB += errorB;
  }

  if(abs(errorB) <  30){
    back_left.move(20);
  }
derivitiveB = errorB - PrevErrorB;
PrevErrorB = errorB;
enValB = back_left.get_position();
errorB = target - enValB;
back_left.move((errorB*kP) + (integralB * kI) + (derivitiveB * kD));
pros::delay(10);
}

int errorC;
int PrevErrorC;
int enValC;
int derivitiveC;
int integralC;


void F_Right(int target, float kP, float kI, float kD, float thresh ){
    if(abs(errorC) > thresh){
      integralC = 0;
    }
    else {
    integralC += errorC;
  }

  if(abs(errorC) < 30){
    front_right.move(20);
  }
derivitiveC = errorC - PrevErrorC;
PrevErrorC = errorC;
enValC = front_right.get_position();
errorC = target - enValC;
front_right.move((errorC*kP) + (integralC * kI) + (derivitiveC * kD));
pros::delay(10);
}

int errorD;
int PrevErrorD;
int enValD;
int derivitiveD;
int integralD;
void B_Right(int target, float kP, float kI, float kD, float thresh ){
    if(abs(errorD) > thresh){
      integralD = 0;
    }
    else {
    integralD += errorD;
  }

    if(abs(errorD) < 30){
      back_right.move(20);
    }
derivitiveD = errorD - PrevErrorD;
PrevErrorD = errorD;
enValD = back_right.get_position();
errorD = target - enValD;
back_right.move((errorD*kP) + (integralD * kI) + (derivitiveD * kD));
pros::delay(10);
}
//End of Drive PID Set Up

void reset(){
arm.set_brake_mode(MOTOR_BRAKE_HOLD);
front_right.set_brake_mode(MOTOR_BRAKE_HOLD);
back_right.set_brake_mode(MOTOR_BRAKE_HOLD);
front_left.set_brake_mode(MOTOR_BRAKE_HOLD);
back_left.set_brake_mode(MOTOR_BRAKE_HOLD);
front_left.tare_position();
back_left.tare_position();
front_right.tare_position();
back_right.tare_position();
enValA = front_left.get_position();
enValB = back_left.get_position();
enValC = front_right.get_position();
enValD = back_right.get_position();
}

//PID Move
float p = 0.55;
float i;
float d;

void PID_Tank(int target_left, int target_right){
  reset();
while((abs(enValA) < (abs(target_left)-5)) && (abs(enValC) < (abs(target_right)-5))){
  F_Left(target_left, p,i,d, 20);
  B_Left(target_left, p,i,d, 20);
  F_Right(target_right, p,i,d, 20);
  B_Right(target_right, p,i,d, 20);
}
  reset();
  pros::delay(10);
}
//PID Tank Delays
void Dlay()/*Positive Value of Left*/{
  while(front_left.get_position() < front_left.get_target_position()){
    pros::delay(1);
  }
  pros::delay(50);
}
void Dlay2()/*Negative Value of Left*/{
  while(front_left.get_position() > front_left.get_target_position()){
    pros::delay(1);
  }
  pros::delay(50);
}
void Tray_Delay(){
	while(tray.get_position() < 4890){
		pros::delay(2);
	}
	pros::delay(50);
}

//Ordinary Functions
void Drive(int l_input, int r_input)/*Controller drive Set Up*/{
	front_right.move(r_input);
	back_right.move(r_input);
	front_left.move(l_input);
	back_left.move(l_input);
}

void Tank(int l, int r, int v)/*Auton Drive STRAIGHT ONLY*/{
front_right.move_relative(r,v);
back_right.move_relative(r, v);
front_left.move_relative(l,v);
back_left.move_relative(l,v);
}

void intake(int input)/*Intake Set Power*/{
left_intake.move(input);
right_intake.move(input);
}
//More Layered Functions
void Arm()/*Driver Control Arm Preset Code*/{
	if(Master.get_digital(DIGITAL_UP) && check == 0)/*First Arm Set Up  Positon Checks button Pressed and Variable State*/{
		arm.move_absolute(500/*Target position*/, 150/*MOvement Velocity out of 200*/);
		check = true; //Changes Variable for next preset
		if(tray.get_position() <1800)/*Checks Tray Posititon and Moves Tray if it's too low*/{
		tray.move_absolute(1800, 200);/*Values repersent similar to arm move absolute*/
		}
		pros::delay(100);//Delay for break
	}
	else if(Master.get_digital(DIGITAL_UP) && check == 1){
		arm.move_absolute(900, 150);
		check = false; //Changes Variable for next Preset check
	if(tray.get_position() < 1800){
		tray.move_absolute(1800, 200);
	}
	pros::delay(100);
	}
	else if(Master.get_digital(DIGITAL_DOWN))/*Checks if button is pressed down*/{
		arm.move_absolute(0, 200);//MOves arm all the way down
		check = false; //Resets Variable for next use of Presets
	}
}
void Intake(){
	if (Master.get_digital(DIGITAL_L1)){
		intake(127);
	}
	else if(Master.get_digital(DIGITAL_L2)){
		intake(-127);
	}
	else{
		intake(15);
	}
}

void Tray(){//Tray Code Checks tray's position and sets motor power based off interval (step function)
 if(tray.get_position() < fast_val){
	 tray.move(127);
 }
 else if(tray.get_position() < fast_val2){
	 tray.move(107);
 }
 else if(tray.get_position() < slow_val){
	 tray.move(85);
 }
 else if(tray.get_position() < slow_val2){
	 tray.move(75);
 }
 else if(tray.get_position() < final_val){
	 tray.move(60);
 }
}

//Auton Void Function
void blue_back(){
	//Start of Flipout
	reset();
	intake(-127);
	arm.move(40);
	pros::delay(350);
	intake(127);
	pros::delay(200);
	arm.move(-40);
	pros::delay(900);
	//End of Flipout
	//Start of First line
	Tank(2000, 2000, 75);
	Dlay();
	pros::delay(50);
	//End of Line
	PID_Tank(220, -220);// Turns right
	pros::delay(50);
	Tank(200, 200, 72);// drives Forward a smidge
	Dlay();
	pros::delay(500);
	Tank(-300, -300, 90); ///moves back a bit
	Dlay2();
	pros::delay(400);
	PID_Tank(600, -600); //Turns right towards goal
	Tank(900, 900, 90); //Drives towards goal
	Dlay();
	pros::delay(300);
	Tray(); //Move Tray
	Drive(40, 40); //Keep moving toward goal
	Tray_Delay(); //wait until tray places
	pros::delay(200);
	Drive(-60, -60); // moves back
	pros::delay(1750);
	Drive(0, 0);
}
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
while(true){
	tray_pos = tray.get_position();// Constant update of tray's Posititon
	Drive(Master.get_analog(ANALOG_LEFT_Y), Master.get_analog(ANALOG_RIGHT_Y));
	Intake();
	Arm();
	if(Master.get_digital(DIGITAL_R1)){
		Tray();
	}
	else if(Master.get_digital(DIGITAL_R2)){
		tray.move(-127);
	}
	else{
		tray.move(tray_pos - tray.get_position());
	}

}
}
