package org.firstinspires.ftc.teamcode.utils;

/**
 * All Robot Constants contained in a convenient interface,
 * A class must implement this to access any constants.
 */
public interface Constants {
    /** Blue Alliance Tag */
    boolean BLUE_ALLIANCE = true;

    /** Red Alliance Tag */
    boolean RED_ALLIANCE = false;



    /** The Default Controller Axis DeadBand */
    double DEFAULT_DEADBAND = 0.005;

    /** The number of code loops to wait for axis values */
    int LOOPS_TO_WAIT = 5;


    /** The virtual low gear for the robot driving */
    double VIRTUAL_LOW_GEAR = 0.5;

    /** The Maximum Spin Speed of a drive motor in ticks/second */
    int MAX_SPIN_SPEED = 2800;

    /** The Governor for Maximum Speed as a proportion of available power */
    double OVERALL_GOVERNOR = 0.9;

    /** The Governor for Manual Turning Speed as a proportion of available power */
    double MANUAL_TURNING_GOVERNOR = 0.3;

    /** The Governor for Auto Turning Speed as a proportion of available power */
    double AUTO_TURNING_GOVERNOR = 0.3;

    /** The Proportional Constant for PID turning to an angle */
    double TURNING_P = 0.015;

    /** The +/- allowed error for autoAligning in degrees */
    double TURNING_ERROR = 0.5;

    /** The Proportional Coefficient for the Back Left Drive Motor */
    double BACK_LEFT_P = 10.0;

    /** The Integral Coefficient for the Back Left Drive Motor */
    double BACK_LEFT_I = 0.0;

    /** The Dampening Coefficient for the Back Left Drive Motor */
    double BACK_LEFT_D = 0.0;

    /** The Feedforward Coefficient for the Back Left Drive Motor */
    double BACK_LEFT_F = 14.0;

    /** The Proportional Coefficient for the Back Right Drive Motor */
    double BACK_RIGHT_P = 10.0;

    /** The Integral Coefficient for the Back Right Drive Motor */
    double BACK_RIGHT_I = 0.0;

    /** The Dampening Coefficient for the Back Right Drive Motor */
    double BACK_RIGHT_D = 0.0;

    /** The Feedforward Coefficient for the Back Right Drive Motor */
    double BACK_RIGHT_F = 14.0;

    /** The Proportional Coefficient for the Front Left Drive Motor */
    double FRONT_LEFT_P = 10.0;

    /** The Integral Coefficient for the Front Left Drive Motor */
    double FRONT_LEFT_I = 0.0;

    /** The Dampening Coefficient for the Front Left Drive Motor */
    double FRONT_LEFT_D = 0.0;

    /** The Feedforward Coefficient for the Front Left Drive Motor */
    double FRONT_LEFT_F = 14.0;;

    /** The Proportional Coefficient for the Front Right Drive Motor */
    double FRONT_RIGHT_P = 10.0;

    /** The Integral Coefficient for the Front Right Drive Motor */
    double FRONT_RIGHT_I = 0.0;

    /** The Dampening Coefficient for the Front Right Drive Motor */
    double FRONT_RIGHT_D = 0.0;

    /** The Feedforward Coefficient for the Front Right Drive Motor */
    double FRONT_RIGHT_F = 14.0;

    /** Ticks per revolution of a REV thru bore encoder */
    int TICKS_PER_REV = 8192;

    /** Approximate Diameter of our Dead Wheels */
    double DEAD_DIAMETER = 2.5;

    /** The distance between the center of the left and right dead wheels in inches */
    double ROBOT_TRACKWIDTH = 5.8;

    /** Inches driven per encoder tick of a dead wheel */
    double INCHES_PER_TICK = DEAD_DIAMETER * Math.PI / TICKS_PER_REV;

    double DEGREES_PER_TICK = DEAD_DIAMETER * .5 * 360.0 / (TICKS_PER_REV * ROBOT_TRACKWIDTH);

    /** The Multiplier for forward distance tracking */
    double FORWARD_ODOMETRY_CORRECTION = 0.96486;

    /** The Multiplier for strafe distance tracking */
    double STRAFE_ODOMETRY_CORRECTION = 0.964378;

    /** The Multiplier for heading tracking */
    double HEADING_ODOMETRY_CORRECTION = 0.90643294416;

    /** The Proportional Constant for PID spline */
    double SPLINE_P = 0.05;

    /** The allowed error for autonomous pathing in inches */
    double SPLINE_ERROR = 2.0;

    /** The Max Speed for splining */
    double SPLINE_GOVERNOR = 0.6;

    /** The X value of the Scoring Points */
    double SCORING_X = -44.0;

    /** The Y value of the Close Blue Scoring Point */
    double BLUE_SCORING_Y_CLOSE = -42.0;

    /** The Y value of the Middle Blue Scoring Point */
    double BLUE_SCORING_Y_MED = -36.0;

    /** The Y value of the Far Blue Scoring Point */
    double BLUE_SCORING_Y_FAR = -30.0;

    /** The Y value of the Close Red Scoring Point */
    double RED_SCORING_Y_CLOSE = 42.0;

    /** The Y value of the Middle Red Scoring Point */
    double RED_SCORING_Y_MED = 36.0;

    /** The Y value of the Far Red Scoring Point */
    double RED_SCORING_Y_FAR = 30.0;

    /** The X value of the Left Waypoint Points */
    double LEFT_WAYPOINT_X = -16.0;

    /** The X value of the Right Waypoint Points */
    double RIGHT_WAYPOINT_X = 40.0;

    /** The Y value of the Blue Waypoint Points */
    double BLUE_WAYPOINT_Y = 36.0;

    /** The Y value of the Red Waypoint Point */
    double RED_WAYPOINT_Y = -36.0;

    /** The X value of the Intake Points */
    double INTAKE_X = 60.0;

    /** The Y value of the Blue Intake Point */
    double BLUE_INTAKE_Y = 60.0;

    /** The Y value of the Red Intake Point */
    double RED_INTAKE_Y = -60.0;



    /** Servo position for intake actuators when up */
    double INTAKE_UP_POSITION = 0.43;

    /** Servo position for intake actuators when touching floor */
    double INTAKE_DOWN_POSITION = 0.75;



    /** Max power allowed for the shoulder motors */
    double SHOULDER_GOVERNOR = 1.0;

    /** The multiplier for arm manual control */
    double ARM_MANUAL_SCALING = 0.9;

    /** The p constant for arm goToPos */
    double SHOULDER_P = 0.007;

    /** The f constant for arm goToPos */
    double SHOULDER_F = 0.0005;

    /** The d constant for arm goToPos */
    double SHOULDER_D = 0.0001;

    /** Pot Conversion last term */
    double POT_COEFF_A = 5.514E5;

    /** Pot Conversion second-last term */
    double POT_COEFF_B = -7.22E5;

    /** Pot Conversion second term */
    double POT_COEFF_C = 3.154E5;

    /** Pot Conversion first term term */
    double POT_COEFF_D = -4.595E4;

    /** The arm angle to launch the plane at */
    double LAUNCH_ARM_ANGLE = 140.0;

    /** The servo position of the plane launcher when cocked */
    double PLANE_LAUNCHER_COCKED = 0.15;

    /** Max power allowed for the wrist motor */
    double WRIST_GOVERNOR = 1.0;

    /** Wrist Position Proportional Coefficient */
    double WRIST_P = 0.023;

    /** Wrist encoder ticks per full revolution */
    int WRIST_TICKS_PER_REV = 288;

    /** The Wrist Angle Offset at the start of the game */
    double WRIST_OFFSET = 90.0;

    /** THe Wrist Manual Control Multiplier */
    double WRIST_COEFF = 0.5;



    /** The Max Useful Servo Range of the Turny Wrist */
    double MAX_TURNY_WRIST = 0.43;

    /** The open position for the left claw */
    double LEFT_CLAW_OPEN = 0.25;

    /** The open position for the right claw */
    double RIGHT_CLAW_OPEN = 0.223;



    /** X Coordinate of Intake Side April Tags */
    double APRIL_TAG_INTAKE_X = 72;

    /** Y Coordinate of Tag #7 */
    double APRIL_TAG_7_Y = 42;

    /** Y Coordinate of Tag #8 */
    double APRIL_TAG_8_Y = 36;

    /** Y Coordinate of Tag #9 */
    double APRIL_TAG_9_Y = -36;

    /** Y Coordinate of Tag #10 */
    double APRIL_TAG_10_Y = -42;

    /** X Coordinate of Scoring Side April Tags */
    double APRIL_TAG_SCORING_X = -61.5;

    /** Y Coordinate of Tag #1 */
    double APRIL_TAG_1_Y = -42;

    /** Y Coordinate of Tag #2 */
    double APRIL_TAG_2_Y = -36;

    /** Y Coordinate of Tag #3 */
    double APRIL_TAG_3_Y = -30;

    /** Y Coordinate of Tag #4 */
    double APRIL_TAG_4_Y = 30;

    /** Y Coordinate of Tag #5 */
    double APRIL_TAG_5_Y = 36;

    /** Y Coordinate of Tag #6 */
    double APRIL_TAG_6_Y = 42;

    /** The distance in inches from the back cam to the robot's center */
    double BACK_CAM_DIST = 6.0;

    /** The forward distance in inches from the front cam to the robot's center */
    double FRONT_CAM_FORWARD_DIST = 5.5;

    /** The horizontal distance in inches from the front cam to the robot's center */
    double FRONT_CAM_HORIZONTAL_DIST = 4.5;
}