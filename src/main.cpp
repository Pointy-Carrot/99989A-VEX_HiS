#include "main.h"
#include "lemlib/api.hpp"
#include "robodash/api.h"
#include <cstdio>
#include <string>

ASSET(path1_txt);

// config

// pros::Motor intakeR(6, pros::v5::MotorGears::blue);
// pros::Motor intakeL(-16, pros::v5::MotorGears::blue);
pros::MotorGroup intake({-6, 16}, pros::MotorGears::blue);

pros::adi::DigitalOut mogo_mech('H');
pros::adi::DigitalOut arm('G');
pros::adi::DigitalOut intake_lift('A');


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
// optical
pros::Optical sorter(5);
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

bool ejecting = false;
bool red = true;

void eject_ring(){
    ejecting = true;
    intake.move(127);
    pros::delay(60);
    intake.move(-127);
    pros::delay(250);
    intake.move(0);
    pros::delay(100);
    intake.move(127);
    pros::delay(400);
    ejecting = false;
}

enum Color {
    RED,
    BLUE
};

void sort_red(){
    sorter.set_led_pwm(50);
    if(sorter.get_hue() < 20){
        if(!ejecting){
            eject_ring();
        }
    }
}

void sort_blue(){
    sorter.set_led_pwm(100);
    if(sorter.get_hue() > 150 && sorter.get_hue() < 270){
        eject_ring();
    }
}





void red_5_ring(){
    red = true;
    chassis.setPose(-48, 30, 270);

    chassis.moveToPoint(-36, 30, 500, {.forwards = false});
    chassis.turnToHeading(300, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(-24, 24, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    mogo_mech.set_value(true);
    pros::delay(250);
    intake.move(127);
    pros::delay(100);
    chassis.turnToPoint(-24, 52, 500);
    chassis.moveToPoint(-24, 40, 500, {.minSpeed = 60});
    intake.move(127);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(-12, chassis.getPose().y-6, 1000, {.maxSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.moveToPoint(-24, chassis.getPose().y+4, 1000, {.forwards = false});
    chassis.turnToPoint(-12, chassis.getPose().y, 500);
    chassis.moveToPoint(-12, chassis.getPose().y, 2000, {.maxSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.moveToPoint(-24, chassis.getPose().y-2, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.turnToHeading(270, 750);
    chassis.moveToPose(-53, 8, 180, 2000, {.maxSpeed = 80});
    intake_lift.set_value(true);
    chassis.waitUntilDone();
    intake_lift.set_value(false);
    pros::delay(250);
    chassis.moveToPoint(-50, 20, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.turnToPoint(-20, 16, 500);
    chassis.moveToPoint(-28, 16, 1000, {.maxSpeed = 60});
}

void red_solo_AWP(){
    red = true;
    chassis.setPose(-48, -18, 270);
    chassis.moveToPoint(-22, -26, 2000, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    mogo_mech.set_value(true);
    pros::delay(250);
    intake.move(127);
    pros::delay(500);
    chassis.moveToPose(-44, -4, 0, 2000, {.minSpeed = 70});
    intake_lift.set_value(true);
    chassis.waitUntilDone();
    intake_lift.set_value(false);
    pros::delay(1250);
    mogo_mech.set_value(false);
    chassis.turnToHeading(0, 500);
    intake.move(0);
    chassis.moveToPoint(-48, 8, 1000);
    chassis.turnToPoint(-20, 24, 500, {.forwards = false});
    chassis.moveToPoint(-20, 24, 1500, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    mogo_mech.set_value(true);
    pros::delay(250);
    chassis.turnToHeading(0, 750);
    intake.move(127);














    // chassis.turnToHeading(55, 500, {.maxSpeed = 75});
    // chassis.waitUntilDone();
    // arm.set_value(true);
    // pros::delay(250);
    // chassis.turnToHeading(180, 1000, {.maxSpeed = 75});
    // chassis.waitUntilDone();
    // arm.set_value(false);
    // pros::delay(250);
    // chassis.moveToPoint(-56, 4, 2000, {.forwards = false, .maxSpeed = 60});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-56, 0, 1000, {.maxSpeed = 60});
    // chassis.turnToHeading(90, 1000, {.maxSpeed = 75});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-61, 0, 1000, {.forwards = false, .maxSpeed = 60});
    // chassis.waitUntilDone();
    // intake.move(127);
    // pros::delay(500);
    // chassis.moveToPoint(-56, 0, 1000);
    // chassis.turnToHeading(0, 500);
    // chassis.moveToPose(-22, -26, 300, 1500, {.forwards = false});
    // chassis.waitUntil(48);
    // mogo_mech.set_value(true);
    // intake.move(80);
    // pros::delay(250);
    // chassis.turnToHeading(255, 500);
    // chassis.moveToPoint(-52, -32, 1000);
    // intake.move(127);
    // chassis.waitUntilDone();
    // pros::delay(200);
    // chassis.turnToPoint(-24, -48, 1000);
    // chassis.moveToPoint(-24, -48, 2000);
    // chassis.waitUntilDone();
    // pros::delay(250);
    // chassis.turnToHeading(0, 750);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-14, -2, 2000);
}

void blue_5_ring(){
    red = false;
    chassis.setPose(48, 30, 90);

    chassis.moveToPoint(36, 30, 500, {.forwards = false});
    chassis.turnToHeading(60, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(24, 24, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    mogo_mech.set_value(true);
    pros::delay(250);
    intake.move(127);
    pros::delay(100);
    chassis.turnToPoint(24, 52, 500);
    chassis.moveToPoint(24, 40, 500, {.minSpeed = 60});
    intake.move(127);
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(12, chassis.getPose().y-4, 1000, {.maxSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.moveToPoint(24, chassis.getPose().y+6, 1000, {.forwards = false});
    chassis.turnToPoint(12, chassis.getPose().y, 500);
    chassis.moveToPoint(12, chassis.getPose().y, 2000, {.maxSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.moveToPoint(24, chassis.getPose().y-2, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.turnToHeading(90, 750);
    chassis.moveToPose(42, 0, 180, 2000, {.maxSpeed = 80});
    intake_lift.set_value(true);
    chassis.waitUntilDone();
    intake_lift.set_value(false);
    pros::delay(250);
    chassis.moveToPoint(42, 20, 1000, {.forwards = false});
    chassis.turnToPoint(8, 6, 500);
    chassis.moveToPoint(8, 6, 2000, {.maxSpeed = 60});
}

void blue_solo_AWP(){
    red = false;
    chassis.setPose(57, -18, 270);

    chassis.turnToHeading(350, 500, {.maxSpeed = 75});
    chassis.waitUntilDone();
    arm.set_value(true);
    pros::delay(250);
    chassis.turnToHeading(225, 750, {.maxSpeed = 75});
    chassis.waitUntilDone();
    arm.set_value(false);
    pros::delay(250);
    chassis.moveToPoint(54, 4, 2000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveToPoint(54, -2, 1000, {.maxSpeed = 60});
    chassis.turnToHeading(270, 1000, {.maxSpeed = 75});
    chassis.waitUntilDone();
    chassis.moveToPoint(61, -2, 1000, {.forwards = false, .maxSpeed = 30});
    chassis.waitUntilDone();
    intake.move(127);
    pros::delay(500);
    chassis.moveToPoint(56, 0, 1000);
    chassis.turnToHeading(0, 500);
    chassis.moveToPose(22, -28, 60, 1500, {.forwards = false});
    chassis.waitUntil(48);
    mogo_mech.set_value(true);
    intake.move(80);
    pros::delay(250);
    chassis.turnToHeading(105, 500);
    chassis.moveToPoint(48, -36, 1000);
    intake.move(127);
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.turnToPoint(24, -52, 1000);
    chassis.moveToPoint(24, -52, 2000);
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.turnToHeading(0, 750);
    chassis.waitUntilDone();
    chassis.moveToPoint(10, -18, 2000);
}

void red_goal_rush(){
    red = true;
    chassis.setPose(-56, -58, 90);
    
    chassis.moveToPoint(-18, -58, 750);
    chassis.swingToHeading(60, lemlib::DriveSide::LEFT, 500);
    chassis.waitUntilDone();
    arm.set_value(true);
    pros::delay(250);
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 500);
    chassis.moveToPoint(-50, -65, 2000, {.forwards = false});
    chassis.turnToHeading(135, 500, {.minSpeed = 90});
    chassis.waitUntilDone();
    arm.set_value(false);
    pros::delay(250);
    chassis.turnToHeading(270, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPose(-20, -64, 270, 1000, {.forwards = false});
    chassis.waitUntilDone();
    mogo_mech.set_value(true);
    pros::delay(250);
    intake.move(127);
    chassis.turnToPoint(chassis.getPose().x-6, -56, 750);
    chassis.moveToPoint(chassis.getPose().x-6, -56, 1000);
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.turnToHeading(0, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
    chassis.waitUntilDone();
    mogo_mech.set_value(false);
    intake.move(-127);
    chassis.turnToHeading(180, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(chassis.getPose().x, -32, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    mogo_mech.set_value(true);
    intake.move(127);
    pros::delay(250);
    chassis.turnToPoint(-54, -14, 500);
    intake_lift.set_value(true);
    chassis.moveToPoint(-54, -14, 2000, {.minSpeed = 80});
    chassis.waitUntilDone();
    intake_lift.set_value(false);
    pros::delay(350);
    chassis.moveToPoint(chassis.getPose().x+12, chassis.getPose().y-12, 1000, {.forwards = false});
}

void blue_goal_rush(){
    red = false;
    chassis.setPose(54, -30, 270);
    
    chassis.moveToPoint(20, -38, 1000); 
    chassis.turnToHeading(270, 500);
    chassis.waitUntilDone();
    arm.set_value(true);
    pros::delay(250);
    chassis.swingToHeading(15, lemlib::DriveSide::LEFT, 750);
    chassis.waitUntilDone();
    arm.set_value(false);
    pros::delay(250);
    chassis.turnToHeading(135, 750, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(chassis.getPose().x-12, chassis.getPose().y+16, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    mogo_mech.set_value(true);
    pros::delay(250);
    chassis.turnToHeading(180, 500);
    intake.move(127);
    chassis.moveToPoint(chassis.getPose().x+6, -58, 1000);
    pros::delay(250);
    chassis.turnToHeading(270, 500);
    chassis.moveToPoint(24, chassis.getPose().y, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(750);
    chassis.turnToHeading(0, 500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
    chassis.waitUntilDone();
    mogo_mech.set_value(false);
    intake.move(-127);
    pros::delay(250);
    chassis.turnToHeading(180, 750, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
    chassis.moveToPoint(24, -38, 750, {.forwards = false});
    chassis.moveToPoint(24, -28, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    mogo_mech.set_value(true);
    pros::delay(250);
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y-16, 1000);
    chassis.turnToHeading(90, 500);
    chassis.moveToPose(46, -8, 0, 2000, {.minSpeed = 90});
    intake_lift.set_value(true);
    intake.move(127);
    chassis.waitUntilDone();
    intake_lift.set_value(false);
    chassis.moveToPoint(46, chassis.getPose().y-12, 1000, {.forwards = false});
}


void prog_skills(){
    red = true;
}



rd::Selector selector({
    {"Red Solo AWP", &red_solo_AWP},
    {"Red 5 Ring", &red_5_ring},
    {"Red Goal Rush", &red_goal_rush},
    {"Blue Solo AWP", &blue_solo_AWP},
    {"Blue 5 Ring", &blue_5_ring},
    {"Blue Goal Rush", &blue_goal_rush},
    {"Prog Skills", &prog_skills}
});








rd::Console console;



void color_sort(Color color){
    if(color == RED){
        sort_red();
    } else if(color == BLUE){
        sort_blue();
    }
}


// initialize function. Runs on program startup
void initialize() {
    chassis.calibrate(); // calibrate sensors
    selector.focus();
    pros::Task sort_task([]{
        while(true){
            if(red){
                color_sort(BLUE);
                pros::delay(20);
            } else if(!red){
                color_sort(RED);
                pros::delay(20);
            }
        }
    });
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
    selector.run_auton();
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
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
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
        
        // manual color sort override
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            red = true;
        } else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            red = false;
        }

        // delay to save resources
        pros::delay(25);

	}
}