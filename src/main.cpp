// N.B. make sure to update trackwidth with teh correct track width
#include "main.h" 
#include "lemlib/pid.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.hpp" // IWYU pragma: keep
#include <unistd.h>
#include <pthread.h>

#define CIRCOMFRENCE 0.0508 // meters

using namespace pros::c;
using namespace pros;
using namespace lemlib;

typedef struct portMaster {
    // 14 integers { sizeof(int) 14 }
    int8_t motor_A; // drivetrain motor
    int8_t motor_B; // drivetrain motor
    int8_t motor_C; // drivetrain motor 
    int8_t motor_D; // drivetrain motor
    int8_t motor_E; // drivetrain motor
    int8_t motor_F; // drivetrain motor
    int8_t motor_Intake; // intake motor
    int8_t motor_H; // top agitator motor
    int8_t motor_I; // bottom agitator motor
    int8_t motor_Scoring; // scoring motor
    int8_t IMU_Sensor; // intertial sensor
    int8_t rotational_Horizontal; // rotational sensor
    int8_t rotational_Vertical; // rotational sensor
    int8_t colour_Sensor; // colour sensor
    char ScraperMech;
} port_t;

static port_t port = {
    .motor_A = 1, // drivetrain motor
    .motor_B = 2, // drivetrain motor
    .motor_C = 3, // drivetrain motor 
    .motor_D = 5,// drivetrain motor
    .motor_E = 9, // drivetrain motor
    .motor_F = 10, // drivetrain motor
    .motor_Intake = 0, // intake motor
    .motor_H = 0, // top agitator motor
    .motor_I = 0, // bottom agitator motor
    .motor_Scoring = 0, // scoring motor
    .IMU_Sensor = 0, // intertial sensor
    .rotational_Horizontal = 18, // rotational sensor
    .rotational_Vertical = 0, // rotational sensor
    .colour_Sensor = 0, // colour sensor
    .ScraperMech = 'A'
};

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

/* deactivate FOR NOW
PID pid(5, // kP
        0.01, // kI
        20, // kD
        5, // integral anti windup range
        false); // don't reset integral when sign of error flips
*/


bool IsAutonomousRunning = true;

pthread_t ClockThread;

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

void DrivetrainSetBrakeMode(motor_brake_mode_e_t mode) {
    motor_set_brake_mode(port.motor_A, mode);
    motor_set_brake_mode(port.motor_B, mode);
    motor_set_brake_mode(port.motor_C, mode);
    motor_set_brake_mode(port.motor_D, mode);
    motor_set_brake_mode(port.motor_E, mode);
    motor_set_brake_mode(port.motor_F, mode);
}

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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

    LEMLIB_INNIT();

    LoadConfig();
    DrivetrainSetBrakeMode(E_MOTOR_BRAKE_BRAKE);
    adi_port_set_config(port.ScraperMech, E_ADI_DIGITAL_OUT);
    imu_reset(port.IMU_Sensor);
    rotation_reset(port.rotational_Horizontal);
    rotation_reset(port.rotational_Vertical);
}

void ScreenSetup() {
    printf("debug");
}

void startup() {
    bool a = true;
    int i = 0;
    while (a == true) {
        if (controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_UP) == 1) {
            i++;
        }
        if (i == 3) {
            a = false;
        }
    }

    i = 0;
    a = true;

    while (a == true) {
        if (controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_DOWN) == 1) { // 
            i++;
        }
        if (i == 3) {
            a = false;
        }
    }
} // PROS API COMPATIBLE

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

void DrivetrainBrake() {
    DrivetrainSetBrakeMode(E_MOTOR_BRAKE_COAST);
    STOP();
    DrivetrainSetBrakeMode(E_MOTOR_BRAKE_BRAKE);
    STOP();
    DrivetrainSetBrakeMode(E_MOTOR_BRAKE_HOLD);
    STOP();
    DrivetrainSetBrakeMode(E_MOTOR_BRAKE_COAST);
} // PROS API COMPATIBLE

void starboard(int32_t a) {
    //Motor1.setVelocity(a, percent);
    motor_move_velocity(port.motor_A, a);
    //Motor2.setVelocity(a, percent);
    motor_move_velocity(port.motor_B, a);
    //Motor3.setVelocity(a, percent);
    motor_move_velocity(port.motor_C, a);
} // PROS API COMPATIBLE

void portside(int32_t a) {
    //Motor9.setVelocity(a, percent);
    motor_move_velocity(port.motor_D, a);
    //Motor10.setVelocity(a, percent);
    motor_move_velocity(port.motor_E, a);
    //Motor5.setVelocity(a, percent);
    motor_move_velocity(port.motor_F, a);
} // PROS API COMPATIBLE

void TURN() { 
  // turn sets the velocites of the drivetrain so that they will tunr in the applied direction
  portside(controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X));
  starboard(controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X));
} // PROS API COMPATIBLE

void MOVE() {
  portside(controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y));
  int tmp = controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y) * -1;
  starboard(tmp);
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

void ScraperMech() {
    static bool ScraperState = false;
    if (ScraperState == false) {
        ScraperState = true;
        adi_digital_write(port.ScraperMech, ScraperState);
    } else {
        ScraperState = false;
        adi_digital_write(port.ScraperMech, ScraperState);
    }
} // PROS API COMPATIBLE

void IntakeControl() {
    float R1Trigger = controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R1);
    float R2Trigger = controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R2);
    if (R1Trigger == true) {
        motor_move_velocity(port.motor_Intake, 127);
    }
    if (R1Trigger == true) {
        motor_move_velocity(port.motor_Intake, -127);
    }
    if (R1Trigger != true && R2Trigger != true) {
        DrivetrainSetBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        motor_brake(port.motor_Intake);
    }
} // PROS API COMPATIBLE

void ScoringControl() {
    float L1Trigger = controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L1);
    float L2Trigger = controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2);

    if (L1Trigger == true) {
        motor_move_velocity(port.motor_Scoring, 127);
    }
    if (L1Trigger == true) {
        motor_move_velocity(port.motor_Scoring, -127);
    }
    if (L1Trigger != true && L2Trigger != true) {
        DrivetrainSetBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        motor_brake(port.motor_Scoring);
    }
} // PROS API COMPATIBLE

void MiscControl() {
    float Ybutton = controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_Y);
    static bool prev = false;
    static bool interupt = false;

    if (prev != Ybutton) {
        interupt = true;
        if (prev == true) {
            prev = false;
        } else {
            prev = true;
        }
    } else {
        interupt = false;
    }

    if (Ybutton == true && interupt == true) {
        ScraperMech();
    }
} // PROS API COMPATIBLE

void AutoMove(uint32_t velocity) {
    rotation_reset(port.rotational_Horizontal);
    rotation_reset(port.rotational_Vertical);
    portside(velocity);
    starboard(velocity);
}

int proportional(float error, float finalDistance, float Kp) {
    return roundf(((error / finalDistance) * 127) * Kp);
}

void *TIMER(void *garbage) {
    for (int i = 0; i < 5; i++) {
        //printf("%d\n", i);
        //fflush(stdout);
        sleep(1);
    }
    IsAutonomousRunning = false;
    return NULL;
}

void CustomAutoUnusedCode() {
    while(IsAutonomousRunning == true) { // run for 15 seconds
        float rotations = (float)rotation_get_position(port.rotational_Horizontal) / 300; // get the amount of rotations
        float DistanceTraveled = rotations * CIRCOMFRENCE; // convert rotations to distance traveled
        float finalDistance = 1.0; // store the target distance
        float error = finalDistance - DistanceTraveled; // compute error value
        
        if (DistanceTraveled < finalDistance) {
            //AutoMove(proportional(error, finalDistance, 0.9));
            //float output = pid.update(error); deactivated for now
            // AutoMove(output); deactivated for now
        } else {
            STOP();
        }
    }
}

/*N.B. Functions Auto_Red_West, Auto_Blue_West and Auto_Blue_east are not finished.
In order to prevent compile time errors and to make them easier to find I have changed their types from void to void(*).
Note to self: Remember to change them to back to void once you have finished programing them.
    ~Tristan
*/

void *Auto_Red_West() {
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
}

void *Auto_Blue_West() {
    return NULL;
}

void *Auto_Blue_East() {
    return NULL;
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

    pthread_create(&ClockThread, NULL, TIMER, NULL);

    
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
  
    //setup code here

    while (true) {
        DRIVE();
        IntakeControl();
        ScoringControl();
        delay(2);
    }
    UpdateConfig();
}
