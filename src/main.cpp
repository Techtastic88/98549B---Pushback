#include "main.h" 
#include "pros/misc.h"
#include "pros/motors.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.hpp" // IWYU pragma: keep
#include <unistd.h>

#define CIRCUMFRENCE 0.259334

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
    int8_t motor_G; // intake motor
    int8_t motor_H; // top agitator motor
    int8_t motor_I; // bottom agitator motor
    int8_t motor_J; // scoring motor
    int8_t IMU_Sensor; // intertial sensor
    int8_t rotational_A; // rotational sensor
    int8_t rotational_B; // rotational sensor
    int8_t colour_Sensor; // colour sensor
} port_t;

static port_t port = {
    .motor_A = 1, // drivetrain motor
    .motor_B = 2, // drivetrain motor
    .motor_C = 3, // drivetrain motor 
    .motor_D = 5,// drivetrain motor
    .motor_E = 9, // drivetrain motor
    .motor_F = 10, // drivetrain motor
    .motor_G = 0, // intake motor
    .motor_H = 0, // top agitator motor
    .motor_I = 0, // bottom agitator motor
    .motor_J = 0, // scoring motor
    .IMU_Sensor = 0, // intertial sensor
    .rotational_B = 0, // rotational sensor
    .colour_Sensor = 0 // colour sensor
};

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({port.motor_A, port.motor_B, port.motor_C}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({port.motor_D, port.motor_E, port.motor_F}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6

static long clock = 0;

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
    FILE* ConfigFile = fopen("/usd/config.bin", "a+");

    if (ConfigFile == NULL) {
        printf("File not detected. Is there an SD card?");
        fclose(ConfigFile);
    } 

    if (ConfigFile != NULL) {
        int items_read = fwrite(&port, sizeof(port_t), 1, ConfigFile);
        fclose(ConfigFile);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    LoadConfig();

    imu_reset(port.IMU_Sensor);
    rotation_reset(port.rotational_A);
    rotation_reset(port.rotational_B);
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


void AutoMove(uint32_t velocity) {
    rotation_reset(port.rotational_A);
    rotation_reset(port.rotational_B);
    portside(velocity);
    starboard(velocity);
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

void TimedAuto() {
    bool a = true;
    int t = 0;
    while(a == true) {
        // moves forward for 15 seconds 
        while (t < 15) { // run for 15 seconds
            AutoMove(100);
        }
        t += 1;
        sleep(1);
    }
}

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
    bool a = true;
    // moves forwards for one meter (inacuarate because there is no PID)
    while (a == true) {
        float rotations = (float)rotation_get_position(port.rotational_A) / 360;
        float DistanceTraveled = rotations * CIRCUMFRENCE;
        if (DistanceTraveled < 1) { // if distance traveled(meters) is less than 1
            AutoMove(100); // move orward for one meter
        }
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
  
    //setup code here

    while (true) {
        DRIVE();
        delay(2);
    }
    UpdateConfig();
}