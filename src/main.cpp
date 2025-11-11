// HI HASAN asfiaeowfjieawifefawf
#include "main.h" 
#include "pros/misc.h"
#include "pros/motors.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.hpp" // IWYU pragma: keep

using namespace pros::c;
using namespace pros;
using namespace lemlib;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({5, 4, 3}, pros::MotorGearset::blue); // left motor group  
pros::MotorGroup rightMotors({6, 9, 7}, pros::MotorGearset::blue); // right motor group

typedef struct MOTOR {
    int port;
    pros::motor_gearset_e_t gear;
} motor_t;

typedef struct portMaster {
    // 14 integers { sizeof(int) 14 }
    int motor_A; // drivetrain motor
    int motor_B; // drivetrain motor
    int motor_C; // drivetrain motor 
    int motor_D; // drivetrain motor
    int motor_E; // drivetrain motor
    int motor_F; // drivetrain motor
    int motor_G; // intake motor
    int motor_H; // top agitator motor
    int motor_I; // bottom agitator motor
    int motor_J; // scoring motor
    int IMU_Sensor;
    int rotational_A;
    int rotational_B;
    int coloor_Sensor;
} port_t;

int* ConfigData;


/*
motor_t motor_A {
    .port = 1,
    .gear = E_MOTOR_GEAR_BLUE
}; // PROS API COMPATIBLE

motor_t motor_B {
    .port = 2,
    .gear = E_MOTOR_GEAR_BLUE
}; // PROS API COMPATIBLE

motor_t motor_C {
    .port = 3,
    .gear = E_MOTOR_GEAR_BLUE
}; // PROS API COMPATIBLE

motor_t motor_D {
    .port = 5,
    .gear = E_MOTOR_GEAR_BLUE
}; // PROS API COMPATIBLE

motor_t motor_E {
    .port = 9,
    .gear = E_MOTOR_GEAR_BLUE
}; // PROS API COMPATIBLE

motor_t motor_F {
    .port = 10,
    .gear = E_MOTOR_GEAR_BLUE
}; // PROS API COMPATIBLE
*/

//pros::Motor(1);
//pros::MotorGroup left_motors({1, 2, 3}); // left motors on ports 1, 2, 3
//pros::MotorGroup right_motors({4, 5, 6}); // right motors on ports 4, 5, 6

void MotorSetup(int portA, int portB, int portC, int portD, int portE, int portF) {

    motor_A.port = portA;
    motor_B.port = portB;
    motor_C.port = portC;
    motor_D.port = portD;
    motor_E.port = portE;
    motor_F.port = portF;
}


/* void LoadConfig() - not finished
void LoadConfig() {
    FILE* ConfigFile = fopen("/usd/config.bin", "a+");



    if (ConfigFile == NULL) {
        printf("File not detected. Is there an SD card?");
        fclose(ConfigFile);
    } 

    if (ConfigFile != NULL) {
        
        for (int i = 0; i < 6; i++) {
            switch (i) {
                case 0:
                    fread(&motor_A.port, sizeof(int), 1, ConfigFile);
                    break;
                case 1:
                    //
                    break;
                case 2:
                    //
                    break;
                case 3:
                    //
                    break;
                case 4:
                    //
                    break;
                case 5:
                    //
                    break;
            }
        }
    }
    
}
*/

void CompileConfig() {
    printf("Debug:Compiling Config\n");
}

void UpdateConfig() {
    printf("Debug:Updating Config\n");
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

/* void MAXSET() - not yet translated to PROS code
void MAXSET() {
  Motor1.setVelocity(100, percent);
  Motor2.setVelocity(100, percent);
  Motor3.setVelocity(100, percent);
  Motor9.setVelocity(100, percent);
  Motor1.setMaxTorque(100, percent);
  Motor2.setMaxTorque(100, percent);
  Motor3.setMaxTorque(100, percent);
  Motor9.setMaxTorque(100, percent);
  Motor15.setVelocity(100, percent);
  Motor16.setVelocity(100, percent);
}
*/

void STOP() {
    //Motor1.stop();
    motor_brake(motor_A.port);
    //Motor2.stop();
    motor_brake(motor_B.port);
    //Motor3.stop();
    motor_brake(motor_C.port);
    //Motor9.stop();
    motor_brake(motor_D.port);
    //Motor10.stop();
    motor_brake(motor_E.port);
    //Motor5.stop();
    motor_brake(motor_F.port);
} // PROS API COMPATIBLE

/*  void FREEZE() - not yet translated to PROS code
void FREEZE() {
  STOP();
  Brain.Screen.print("button pressed");
  wait(5, seconds);
}
*/

/* void DRIVE() - obsolete
void DRIVE() {
  // drive moves the drivetrain (is dependant on the direction set by TURN() and MOVE())
  //Motor1.spin(forward);
  //Motor2.spin(forward);
  //Motor3.spin(forward);
  //Motor9.spin(forward);
  //Motor10.spin(forward);
  //Motor5.spin(forward);
}
*/

void starbord(int32_t a) {
    //Motor1.setVelocity(a, percent);
    motor_move_velocity(.motor_A, a);
    //Motor2.setVelocity(a, percent);
    motor_move_velocity(motor_B, a);
    //Motor3.setVelocity(a, percent);
    motor_move_velocity(motor_C, a);
} // PROS API COMPATIBLE

void portside(int32_t a) {
    //Motor9.setVelocity(a, percent);
    motor_move_velocity(motor_D, a);
    //Motor10.setVelocity(a, percent);
    motor_move_velocity(motor_E, a);
    //Motor5.setVelocity(a, percent);
    motor_move_velocity(motor_F, a);
} // PROS API COMPATIBLE

void TURN() { 
  // turn sets the velocites of the drivetrain so that they will tunr in the applied direction
  portside(controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X));
  starbord(controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X));
} // PROS API COMPATIBLE

void MOVE() {
  portside(controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y));
  int tmp = controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y) * -1;
  starbord(tmp);
} // PROS API COMPATIBLE

void DRIVE() {
    //check if turning joystick is active
    if (controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X) != 0) {
        TURN();
    } else {
        if (controller_get_analog(E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y) != 0) {
            MOVE();
        }
    }
} // PROS API COMPATIBLE

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    imu_reset(IMU_Sensor);
    rotation_reset(rotational_A);
    rotation_reset(rotational_B);
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

}