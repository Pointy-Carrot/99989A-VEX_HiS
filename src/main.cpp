#include "main.h"
#include "lemlib/api.hpp"
#include "robodash/api.h"



// config

// pros::Motor intakeR(6, pros::v5::MotorGears::blue);
// pros::Motor intakeL(-16, pros::v5::MotorGears::blue);
pros::MotorGroup intake({6, -16}, pros::MotorGears::blue);

pros::adi::DigitalOut mogo_mech('H');
pros::adi::DigitalOut arm('G');


// left motor group
pros::MotorGroup left_motor_group({-20, -19, -18}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({10, 9, 8}, pros::MotorGears::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              14.5, // 14.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              8 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(7);
// horizontal tracking wheel encoder
// pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
// pros::adi::Encoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);



void red_left_qual_auto(){
    chassis.setPose(-48, 30, 270);

    chassis.moveToPoint(-35, 30, 500, {.forwards = false});
    chassis.turnToHeading(300, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(chassis.getPose().x+6, chassis.getPose().y-8, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    mogo_mech.set_value(true);
    pros::delay(500);
    intake.move(127);
    chassis.turnToPoint(-24, 52, 500);
    chassis.moveToPoint(-24, 40, 500, {.minSpeed = 60});
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 500);
    chassis.waitUntilDone();
    intake.move(-127);
    pros::delay(250);
    intake.move(127);
    pros::delay(250);
    chassis.moveToPoint(-4, chassis.getPose().y-10, 1000, {.maxSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.moveToPoint(-24, chassis.getPose().y+8, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(250);
    intake.move(-127);
    pros::delay(250);
    intake.move(127);
    chassis.turnToPoint(-12, chassis.getPose().y+2, 500);
    chassis.moveToPoint(-12, chassis.getPose().y+2, 2000, {.maxSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.moveToPoint(-24, chassis.getPose().y-2, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.turnToPoint(-16, 16, 500);
    chassis.moveToPoint(-16, 16, 1000, {.maxSpeed = 60});
    chassis.waitUntil(24);
    mogo_mech.set_value(false);
}

void red_right_qual_auto(){

}

void blue_left_qual_auto(){

}

void blue_right_qual_auto(){

}



void red_left_elim_auto(){

}

void red_right_elim_auto(){

}

void blue_left_elim_auto(){

}

void blue_right_elim_auto(){

}


void prog_skills(){

}



rd::Selector selector({
    {"Red Qual Left", &red_left_qual_auto},
    {"Red Qual Right", &red_right_qual_auto},
    {"Blue Qual Left", &blue_left_qual_auto},
    {"Blue Qual Right", &blue_right_qual_auto},
    {"Red Elim Left", &red_left_elim_auto},
    {"Red Elim Right", &red_right_elim_auto},
    {"Blue Elim Left", &blue_left_elim_auto},
    {"Blue Elim Left", &blue_right_elim_auto},
    {"Prog Skills", &prog_skills}
});








rd::Console console;






// initialize function. Runs on program startup
void initialize() {
    chassis.calibrate(); // calibrate sensors
    selector.focus();
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
void autonomous() {
    red_left_qual_auto();
}

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
	console.focus();
	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	bool ring_mech_on = false;
    intake.set_brake_mode(pros::MotorBrake::coast);

	while (true) {
		// get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

		// intake controls
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
			intake.move(127);
            ring_mech_on = true;
		} else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
            if(ring_mech_on){
                intake.move(0);
                ring_mech_on = false;
            } else {
                intake.move(-127);
                ring_mech_on = true;
            }
		}
		
		// mogo mech controls
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
			mogo_mech.set_value(true);
		} else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			mogo_mech.set_value(false); 
		}

        // arm controls
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            arm.set_value(true);
        } else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            arm.set_value(false);
        }

        // delay to save resources
        pros::delay(25);

	}
}