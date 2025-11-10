#include "main.h"
#include "pros/misc.h"

typedef struct motor {
    
} motor_t;

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


void starbord(int a) {
  Motor1.setVelocity(a, percent);
  Motor2.setVelocity(a, percent);
  Motor3.setVelocity(a, percent);
}

void portside(int a) {
  Motor9.setVelocity(a, percent);
  Motor10.setVelocity(a, percent);
  Motor5.setVelocity(a, percent);
}

void STOP() {
  Motor1.stop();
  Motor2.stop();
  Motor3.stop();
  Motor9.stop();
  Motor10.stop();
  Motor5.stop();
}

void FREEZE() {
  STOP();
  Brain.Screen.print("button pressed");
  wait(5, seconds);
}

void DRIVE() {
  // drive moves the drivetrain (is dependant on the direction set by TURN() and MOVE())
  Motor1.spin(forward);
  Motor2.spin(forward);
  Motor3.spin(forward);
  Motor9.spin(forward);
  Motor10.spin(forward);
  Motor5.spin(forward);
}

void TURN() { 
  // turn sets the velocites of the drivetrain so that they will tunr in the applied direction
  portside(Controller1.Axis1.position());
  starbord(Controller1.Axis1.position());
}

void MOVE() {
  portside(Controller1.Axis3.position());
  int tmp = Controller1.Axis3.position() * -1;
  starbord(tmp);
}

void heading_Y() {
  MOVE();
  DRIVE();
}

void heading_X() {
  TURN();
  DRIVE();

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

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
  
    //setup code here

    while (true) {
    //loop code
    delay(2);
    }
    }