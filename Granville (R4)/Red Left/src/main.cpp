#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

// motor groups
pros::Controller master(pros::E_CONTROLLER_MASTER);



pros::MotorGroup rightMotors({20, 19, 18}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup leftMotors({-15, -16, -17}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)



// ------------ //
// Other Motors //
// ------------ //

pros::MotorGroup intake({10, -9}, pros::MotorGearset::blue);



// pros:: Motor smthing else(+-PORT, MotorGearset);

// ---------- //
// PNEUMATICS //
// ---------- //

pros::adi::DigitalOut mogo_mech ('H');



// Inertial Sensor on port 5
pros::Imu imu(5);

// ------------ //
// ODOM SENSORS //
// ------------ //

// TODO: init odom sensors (not on robot yet)

pros::Rotation horizontal_encoder(3);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_325, +3.75); //just add this for now, if we decide
                                                                                                         //to put odom wheel(s) then this will be good

// tracking center:
    // 8.75, 7.25

// total chassis
    // 17.5, 14.5

// horizontal wheel
    // 7.6, 11
    
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.6, // 12.6 inch track width
                              lemlib::Omniwheel::OLD_325, // using new 4" omnis
                              360, // drivetrain rpm is 200 (green direct) – adjust this if you can tell what the rpm is
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8 – we have traction so its 8
);


// lateral motion controller
lemlib::ControllerSettings linearController(20, //13.5, // proportional gain (kP) – this should be fine, if it still slightly oscillates reduce it a little
                                              0.004, //0.1, // integral gain (kI) – this should be fine too, again, refer to programming last two pages how to tune PID
                                              122.5, //3, // derivative gain (kD) – same as above (this should be fine too)
                                              5, // anti windup
                                              0.5, // small error range, in inches
                                              1000, // small error range timeout, in milliseconds
                                              2, // large error range, in inches
                                              1000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// angular motion controller



lemlib::ControllerSettings angularController(3.3, //2.5, // proportional gain (kP)
                                              0.054, //0.03, // integral gain (kI)
                                              19.75, //3, // derivative gain (kD)
                                              5, // anti windup
                                              0.5, // small error range, in inches
                                              1000, // small error range timeout, in milliseconds
                                              2, // large error range, in inches
                                              1000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
); 

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel, set to nullptr bc we don't have
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have
                            nullptr, // &horizontal_tracking_wheel, set to nullptr bc we don't have
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have
                            &imu // inertial sensor &imu
);     //doesnt have anything bc we dont have odom wheels

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void auton_init()
{
    
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.calibrate(); // calibrate sensors
    // Additional auton init code
}


void controller_controls()
{
    
float leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
float rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
//     // move the chassis with curvature drive
// chassis.tank(-rightY, leftY, 0.7);

//     leftMotors.move(-rightY);
//     rightMotors.move(leftY);

chassis.tank(leftY, rightY, false);
    
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::Task screenTask([]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
    auton_init();
    // set position to x:0, y:0, heading:0
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() 
{}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    auton_init();
    // set position to x:0, y:0, heading:0
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0, 0, 0);
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
void autonomous() 
{
    // turn to face heading 90 with a very long timeout
    // chassis.turnToHeading(180, 10000);
    // pros::c::delay(3000);
    // chassis.turnToHeading(0, 10000);
    // chassis.moveToPoint(0, 24, 1000);
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0.0, 27.0, 0, 4000, { .forwards = false }, false);
    mogo_mech.set_value(true);

    intake.move(127);
    pros::delay(3000);
    

    chassis.moveToPose(13.0, 40.0, -45, 4000, {.forwards = false}, false);
}   //same thing with blue right, expand to intake more rings on stake and then touch the ladder

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


/**
 * Runs in driver control
 */
void opcontrol() 
{
    // controller
    // loop to continuously update motors

    // pros::Controller master(pros::E_CONTROLLER_MASTER);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


    bool left_spinning = true;
    bool right_spinning = true;


    bool intake_spinning = true;

    bool toggle = false;
    
    while (true) {

        // --------------- //
        // intake controls //
        // --------------- //

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intake.move(-127); // Spin forward
            intake_spinning = false;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            intake.move(127); // Spin reverse
            intake_spinning = false;
        }
        else if (intake_spinning == false)
        {
            intake.brake();
            intake_spinning = true;
        }
        // --------------- // 



        // --------------- //
        //  Mogo Mech Ctl  //
        // --------------- //

        // When button pressed
            // If button released, run following

        bool b_button = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        if (toggle)
        {
            mogo_mech.set_value(true); // turns clamp solenoid on
        }
        else 
        {
            mogo_mech.set_value(false); // turns clamp solenoid off
        }

        

        

        
        // bool R2_toggle = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2);
        // if (R2_toggle) {
        //     toggle = !toggle;
        //     mogo_mech.set_value(toggle);
        // }

        // while (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) // Mogo Mech
        // {
        //     if (mogo_pis)
        //     {
        //         mogo_mech.set_value(true); // Retract piston
        //         mogo_pis = !mogo_pis;
        //         pros::delay(400);
        //     }
        //     else
        //     {
        //         mogo_mech.set_value(false); // Extend piston
        //         mogo_pis = !mogo_pis;
        //         pros::delay(400);
        //     }            
        // }
        
        // --------------- // 


        // --------------- //
        //   Endgame Ctl   //
        // --------------- //

        
        // When button pressed
            // If button released, run following

        // ---------------- //
        //  Controller Ctl  //
        // ---------------- //
        
        controller_controls();
        
        // --------------- // 
        pros::delay(10);
         
    }
}



// void fastTurn(std::string direction, int degrees) {
//     // Conversion factor from degrees to motor rotation
//     const double degToRot = 0.01745;  // Example conversion rate, adjust as needed
    
//     // Calculate the target rotation for motors
//     double targetRotation = degrees * degToRot;
    
//     // Check direction and apply appropriate motor speeds
//     if (direction == "left") {
//         // Left turn: left motors spin backward, right motors spin forward
//         // lemlib::Drivetrain::setMotorVoltage(-12000, 12000);  // Set voltage to max (-12000 for left, 12000 for right)
//         leftMotors.move(-12000);
//         rightMotors.move(12000);
//     } 
//     else if (direction == "right") {
//         // Right turn: left motors spin forward, right motors spin backward
//         // lemlib::Drivetrain::setMotorVoltage(12000, -12000);  // Set voltage to max (12000 for left, -12000 for right)
//         leftMotors.move(12000);
//         rightMotors.move(-12000);
//     }
    
//     // Use encoders to track the rotation progress
//     while (imu.get_heading() < targetRotation) {
//         // Continue turning until desired degrees are met
//         pros::delay(10);  // Small delay to prevent overload
//     }

//     // Stop the motors after reaching the target rotation
//     // lemlib::Drivetrain::setMotorVoltage(0, 0);
//     leftMotors.move(0);
//     rightMotors.move(0);
// }