#include "main.h"
//#include "lemlib/pid.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.hpp" // IWYU pragma: keep
#include "pros/optical.h"
#include <cstdint>
#include <cstdio>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>

using namespace pros::c;
using namespace pros;

typedef struct portMaster {
    // 16 8-bit integers and one char (also 8 bits) total size = 128 bits
    int8_t motor_A; // drivetrain motor
    int8_t motor_B; // drivetrain motor
    int8_t motor_C; // drivetrain motor 
    int8_t motor_D; // drivetrain motor
    int8_t motor_E; // drivetrain motor
    int8_t motor_F; // drivetrain motor
    int8_t motor_Intake; // intake motor
    int8_t motor_Sort; // colour sorting motor
    int8_t motor_I; // bottom agitator motor
    int8_t motor_Scoring; // scoring motor
    int8_t IMU_Sensor; // intertial sensor
    int8_t rotational_Horizontal; // rotational sensor
    int8_t rotational_Vertical; // rotational sensor
    int8_t colour_Sensor; // colour sensor
    char ScraperMech;
    int8_t autoInfo;
} port_t;

static port_t port = {
    .motor_A = 1, // drivetrain motor
    .motor_B = 2, // drivetrain motor
    .motor_C = 3, // drivetrain motor 
    .motor_D = 5,// drivetrain motor
    .motor_E = 9, // drivetrain motor
    .motor_F = 10, // drivetrain motor
    .motor_Intake = 0, // intake motor
    .motor_Sort = 12, // colour sorting motor
    .motor_I = 0, // bottom agitator motor
    .motor_Scoring = 0, // scoring motor
    .IMU_Sensor = 0, // intertial sensor
    .rotational_Horizontal = 18, // rotational sensor
    .rotational_Vertical = 0, // rotational sensor
    .colour_Sensor = 0, // colour sensor
    .ScraperMech = 'A',
    .autoInfo = 1

};

#pragma region Lemlib:

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motor_group({port.motor_A, port.motor_B, port.motor_C}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup right_motor_group({port.motor_D, port.motor_E, port.motor_F}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6

pros::Imu imu(port.IMU_Sensor);

pros::Rotation horizontalEnc(port.rotational_Horizontal);
pros::Rotation verticalEnc(port.rotational_Vertical);

lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::OLD_275, 0);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);

lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                            &right_motor_group, // right motor group
                            10, // 10 inch track width 
                            lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                            450, // drivetrain rpm is 450
                            2 // horizontal drift is 2 (for now)
);
// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);
// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            10, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in degrees
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in degrees
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void LEMLIB_INNIT() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

#pragma endregion Lemlib





#pragma region Drivetrain:

void STOP() {
    //Motor1.stop();
    motor_brake(port.motor_A);
    //Motor2.stop();
    motor_brake(port.motor_B);
    //Motor3.stop();
    motor_brake(port.motor_C);
    //Motor9.stop();
    motor_brake(port.motor_D);
    //Motor10.stop();
    motor_brake(port.motor_E);
    //Motor5.stop();
    motor_brake(port.motor_F);
} // PROS API COMPATIBLE

void DrivetrainInnit() {
	motor_set_gearing(port.motor_A, E_MOTOR_GEARSET_06);
  	motor_set_encoder_units(port.motor_A, E_MOTOR_ENCODER_DEGREES);

	motor_set_gearing(port.motor_B, E_MOTOR_GEARSET_06);
  	motor_set_encoder_units(port.motor_B, E_MOTOR_ENCODER_DEGREES);

	motor_set_gearing(port.motor_C, E_MOTOR_GEARSET_06);
  	motor_set_encoder_units(port.motor_C, E_MOTOR_ENCODER_DEGREES);

	motor_set_gearing(port.motor_D, E_MOTOR_GEARSET_06);
  	motor_set_encoder_units(port.motor_D, E_MOTOR_ENCODER_DEGREES);

	motor_set_gearing(port.motor_E, E_MOTOR_GEARSET_06);
  	motor_set_encoder_units(port.motor_E, E_MOTOR_ENCODER_DEGREES);

	motor_set_gearing(port.motor_F, E_MOTOR_GEARSET_06);
  	motor_set_encoder_units(port.motor_F, E_MOTOR_ENCODER_DEGREES);
}

void starboard(int32_t a) {
    //Motor1.setVelocity(a, percent);
    motor_move_velocity(port.motor_A, a *(600/127));
    //Motor2.setVelocity(a, percent);
    motor_move_velocity(port.motor_B, a *(600/127));
    //Motor3.setVelocity(a, percent);
    motor_move_velocity(port.motor_C, a *(600/127));
} // PROS API COMPATIBLE

void portside(int32_t a) {
    //Motor9.setVelocity(a, percent);
    motor_move_velocity(port.motor_D, a *(600/127));
    //Motor10.setVelocity(a, percent);
    motor_move_velocity(port.motor_E, a *(600/127));
    //Motor5.setVelocity(a, percent);
    motor_move_velocity(port.motor_F, a *(600/127));
} // PROS API COMPATIBLE

void TURN() { 
  // turn sets the velocites of the drivetrain so that they will tunr in the applied direction
  portside(controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X) * -1);
  starboard(controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X) * -1);
} // PROS API COMPATIBLE

void MOVE() {
    int tmp = controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y);
    portside(tmp);
    int tmp2 = controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y);
    starboard(tmp2);
} // PROS API COMPATIBLE

void DRIVE() {
    //check if turning joystick is active
    if (controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X) != 0) {
        TURN();
    } else {
        if (controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y) != 0) {
            MOVE();
        } else {
            STOP();
        }
    }
} // PROS API COMPATIBLE

#pragma endregion Drivetrain





#pragma region Misc

void LoadConfig() {
    FILE* ConfigFile = fopen("/usd/config.bin", "a+");

    if (ConfigFile == NULL) {
        printf("File not detected. Is there an SD card?");
        fclose(ConfigFile);
    } 

    if (ConfigFile != NULL) {
        int items_read = fread(&port, sizeof(port_t), 1, ConfigFile);
        fclose(ConfigFile);
    }
}

void UpdateConfig() {
    FILE* ConfigFile = fopen("/usd/config.bin", "w");

    if (ConfigFile == NULL) {
        printf("File not detected. Is there an SD card?");
        fclose(ConfigFile);
    } 

    if (ConfigFile != NULL) {
        int items_read = fwrite(&port, sizeof(port_t), 1, ConfigFile);
        fclose(ConfigFile);
    }
}

void ScraperMech() {
    static bool ScraperState = false;
	int32_t Ybutton = controller_get_digital_new_press(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_A);
	if (Ybutton == 1) {
		ScraperState = !ScraperState;
    	adi_digital_write(port.ScraperMech, ScraperState);
	}
	
} // PROS API COMPATIBLE

void IntakeControl() {
    int32_t R1Trigger = controller_get_digital_new_press(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R1);
    int32_t R2Trigger = controller_get_digital_new_press(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R2);
    if (R1Trigger == 1) {
        motor_move(port.motor_Intake, 127);
    }
    if (R2Trigger == 1) {
        motor_move(port.motor_Intake, -127);
    }
    if (R1Trigger != 1 && R2Trigger != 1) {
        motor_move(port.motor_Intake, 0);
    }
} // PROS API COMPATIBLE

void ColorSort() {
	optical_rgb_s_t RGB_Values = optical_get_rgb(port.colour_Sensor);
    if (RGB_Values.blue > 0) { // if blue
        delay(50);
        motor_move(port.motor_Sort, -127);
    }
    if (RGB_Values.red > 0) { // if red 
		delay(50);
        motor_move(port.motor_Sort, 127);
    }
}

void ScoringControl() {
    int32_t L1Trigger = controller_get_digital_new_press(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L1);
    int32_t L2Trigger = controller_get_digital_new_press(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2);
    if (L1Trigger == 1) {
        motor_move(port.motor_Scoring, 127);
    }
    if (L2Trigger == 1) {
        motor_move(port.motor_Scoring, -127);
    }
    if (L1Trigger != 1 && L2Trigger != 1) {
        motor_move(port.motor_Scoring, 0);
    }
} // PROS API COMPATIBLE

#pragma endregion Misc





#pragma region Autonomous:

void *Auto_Red_West() {
    chassis.moveToPoint(0, 0, 5000);
    chassis.moveToPoint(6.87, 34.582, 5000);
    chassis.moveToPoint(6.832, 42.085, 5000);
    chassis.moveToPoint(32.103, 10.807, 5000);
    chassis.moveToPoint(31.974, 3.279, 5000);
    chassis.moveToPoint(41.573, 16.71, 5000);
    chassis.moveToPoint(49.19, 16.943, 5000);
    chassis.moveToPoint(31.371, 29.554, 5000);
    chassis.moveToPoint(31.644, 37.662, 5000);
    return NULL;
}

void Auto_Red_East() {
    chassis.moveToPoint(-600, -600, 4000);
    chassis.moveToPoint(-1200, -600, 4000, {.forwards = false});

    chassis.turnToHeading(90, 4000);

    chassis.moveToPoint(-1200, -1200, 4000);

    chassis.turnToHeading(90, 4000);
    // TODO move scraper down
    // TODO run intake
    chassis.moveToPoint(-1600, -1200, 4000);

    // generated 

    chassis.moveToPoint(0, 0, 5000);
    chassis.moveToPoint(-6.87, 34.582, 5000);
    chassis.moveToPoint(-6.832, 42.085, 5000);
    chassis.moveToPoint(-32.103, 10.807, 5000);
    chassis.moveToPoint(-31.974, 3.279, 5000);
    chassis.moveToPoint(-41.573, 16.71, 5000);
    chassis.moveToPoint(-49.19, 16.943, 5000);
    chassis.moveToPoint(-31.371, 29.554, 5000);
    chassis.moveToPoint(-31.644, 37.662, 5000);

}

void *Auto_Blue_West() {
    chassis.moveToPoint(0, 0, 5000);
    chassis.moveToPoint(6.87, 34.582, 5000);
    chassis.moveToPoint(6.832, 42.085, 5000);
    chassis.moveToPoint(32.103, 10.807, 5000);
    chassis.moveToPoint(31.974, 3.279, 5000);
    chassis.moveToPoint(41.573, 16.71, 5000);
    chassis.moveToPoint(49.19, 16.943, 5000);
    chassis.moveToPoint(31.371, 29.554, 5000);
    chassis.moveToPoint(31.644, 37.662, 5000);
    return NULL;
}

void *Auto_Blue_East() {
    chassis.moveToPoint(0, 0, 5000);
    chassis.moveToPoint(-6.87, 34.582, 5000);
    chassis.moveToPoint(-6.832, 42.085, 5000);
    chassis.moveToPoint(-32.103, 10.807, 5000);
    chassis.moveToPoint(-31.974, 3.279, 5000);
    chassis.moveToPoint(-41.573, 16.71, 5000);
    chassis.moveToPoint(-49.19, 16.943, 5000);
    chassis.moveToPoint(-31.371, 29.554, 5000);
    chassis.moveToPoint(-31.644, 37.662, 5000);
    return NULL;
}

#pragma endregion





#pragma region PROS tasks

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
	LoadConfig(); // load the configuration file
	DrivetrainInnit(); // intitialize the drivetrain motors
	optical_set_led_pwm(port.colour_Sensor, 100); // set the colour sensor led to 100%
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
void autonomous() {
	switch (port.autoInfo) {
        case 0:
            Auto_Red_West();
            break;
        case 1:
            Auto_Red_East();
            break;
        case 2:
            Auto_Blue_West();
            break;
        case 3:
            Auto_Blue_East();
            break;
    }
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		/*
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		*/

		DRIVE();
		IntakeControl();
		ScoringControl();
		ScraperMech();
		ColorSort();

		pros::delay(20); // Run for 20 ms then update
	}
}

#pragma endregion PROS tasks