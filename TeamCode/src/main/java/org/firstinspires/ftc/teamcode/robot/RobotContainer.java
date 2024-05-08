package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controller.GameController;
import org.firstinspires.ftc.teamcode.utils.Constants;

//TODO: test and troubleshoot full blue AND RED alliance controls, including drone launch
//TODO: autos
//TODO: drive practice
//TODO: add back pot functionalities eventually

/**
 * The Robot Container. Contains the robot.
 */
public class RobotContainer implements Constants {
    private final boolean alliance;
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Arm arm;
    private final Hand hand;
    private final IndicatorLights lights;
    private final Vision vision;
    private final GameController driverOI;
    private final GameController operatorOI;
    private final Telemetry telemetry;
    private double power;
    private double angle;
    private double turn;
    private boolean fieldOriented;
    private boolean autoAlign;
    private boolean splineToIntake;
    private boolean splineToScoring;
    private double splineScoringY;
    private boolean fourbar;

    /**
     * Initialize the RobotContainer object
     *
     * @param hwMap the hardWareMap
     * @param telemetry the telemetry
     * @param alliance  the alliance
     * @param x starting x
     * @param y starting y
     */
    public RobotContainer(HardwareMap hwMap, Telemetry telemetry, boolean alliance, double x, double y, double heading, Gamepad g1, Gamepad g2, double clawStartingPos) {
        this.alliance = alliance;
        this.fieldOriented = true;
        this.autoAlign = true;
        this.fourbar = true;

        drivetrain = new Drivetrain(hwMap, alliance, x, y, heading);
        intake = new Intake(hwMap);
        arm = new Arm(hwMap);
        hand = new Hand(hwMap, clawStartingPos);
        hand.setClawPos(1.0);
        lights = new IndicatorLights(hwMap, alliance);
        vision = new Vision(hwMap);

        driverOI = new GameController(g1);
        operatorOI = new GameController(g2);

        this.telemetry = telemetry;
    }

    /**
     * Robots. Call in each loop() of the teleop.
     */
    public void robot() {
        updateInstances();

        driverControls();
        operatorControls();

        setLightsColor();
        printTelemetry();
    }

    private void updateInstances() {
        driverOI.updateValues();
        operatorOI.updateValues();

        //arm.update();
        drivetrain.updateWithOdometry();

//        if(!driverOI.start.get() && !operatorOI.start.get())
//            return;
        double[] pose = vision.update();
        if(pose != null)
            drivetrain.updateWithAprilTags(pose);
    }

    private void driverControls() {
        intake.setPanPos(driverOI.right_trigger.get() == 0 );

//        if(driverOI.left_bumper.get() )
//            arm.setExtensionPos(1.0);
//        else if(driverOI.right_bumper.get() )
//            arm.setExtensionPos(0.0);

        power = driverOI.leftStickRadius();
        angle = driverOI.leftStickTheta(alliance);
        turn = driverOI.right_stick_x.get();

        if(driverOI.left_trigger.get() > 0.0) {
            power *= VIRTUAL_LOW_GEAR;
            turn *= VIRTUAL_LOW_GEAR;
        }

        autoAlign = driverOI.right_stick_x.wasZeroLongEnough();

        if(!autoAlign)
            drivetrain.setDesiredHeading(drivetrain.getFieldHeading());
        else if(driverOI.y.wasJustPressed())
            drivetrain.setDesiredHeading(alliance ? 90.0 : -90.0);
        else if(driverOI.a.wasJustPressed())
            drivetrain.setDesiredHeading(alliance ? -90.0 : 90.0);
        else if(driverOI.b.wasJustPressed())
            drivetrain.setDesiredHeading(alliance ? 0.0 : -180.0);
        else if(driverOI.x.wasJustPressed())
            drivetrain.setDesiredHeading(alliance ? -180.0 : 0.0);

        fieldOriented = !driverOI.back.getToggleState();

        if(alliance)
            blueSplineControls();
        else
            redSplineControls();

        if(power != 0.0 || driverOI.left_stick_button.get()) {
            splineToScoring = false;
            splineToIntake = false;
        }

        if(splineToIntake)
            drivetrain.splineToIntake(turn, autoAlign);
        else if(splineToScoring)
            drivetrain.splineToScoring(turn, autoAlign, splineScoringY);
        else
            drivetrain.drive(power, angle, turn, autoAlign, fieldOriented);
    }

    private void blueSplineControls() {
        if(driverOI.dpad_left.wasJustPressed()) {
            splineScoringY = BLUE_SCORING_Y_MED;
            splineToScoring = true;
        }
        else if(driverOI.dpad_right.wasJustPressed())
            splineToIntake = true;

        else if(driverOI.dpad_up.wasJustPressed()) {
            splineScoringY = BLUE_SCORING_Y_FAR;
            splineToScoring = true;
        }
        else if(driverOI.dpad_down.wasJustPressed()) {
            splineScoringY = BLUE_SCORING_Y_CLOSE;
            splineToScoring = true;
        }
    }

    private void redSplineControls() {
        if(driverOI.dpad_left.wasJustPressed())
            splineToIntake = true;

        else if(driverOI.dpad_right.wasJustPressed()) {
            splineScoringY = RED_SCORING_Y_MED;
            splineToScoring = true;
        }
        else if(driverOI.dpad_up.wasJustPressed()) {
            splineScoringY = RED_SCORING_Y_FAR;
            splineToScoring = true;
        }
        else if(driverOI.dpad_down.wasJustPressed()) {
            splineScoringY = RED_SCORING_Y_CLOSE;
            splineToScoring = true;
        }
    }

    private void operatorControls() {
//        double armDesiredAngle;
//        if(operatorOI.dpad_up.get())
//            armDesiredAngle = 180.0;
//        else if(operatorOI.dpad_down.get())
//            armDesiredAngle = 30.0;
//        else if(operatorOI.dpad_left.get() && alliance || operatorOI.dpad_right.get() && !alliance)
//            armDesiredAngle = 90.0;
//        else if(operatorOI.dpad_left.get() && !alliance || operatorOI.dpad_right.get() && alliance)
//            armDesiredAngle = LAUNCH_ARM_ANGLE;
//        else
//            armDesiredAngle = arm.getArmAngle();
//
//        if(intake.getPosition() <= 0.59 && arm.getArmAngle() <= 40.0 &&
//                (operatorOI.left_stick_y.get() >= 0.0 || armDesiredAngle > arm.getArmAngle()) )
//            arm.armManualControl(0.0);
//        else if(operatorOI.left_stick_y.get() == 0.0)
//            arm.armGoToPos(armDesiredAngle);
//        else
            arm.armManualControl(operatorOI.left_stick_y.get() * ARM_MANUAL_SCALING);

            if(operatorOI.a.wasJustPressed())
                fourbar = true;

            if(operatorOI.right_stick_y.get() != 0.0)
                fourbar = false;

            if(fourbar)
                arm.setWristPos(85.0);
            else
                arm.wristManualControl(operatorOI.right_stick_y.get() * WRIST_COEFF);

//        if(operatorOI.a.wasJustPressed())
//            fourbar = true;
//
//        if(operatorOI.right_stick_y.get() != 0.0) {
//            fourbar = false;
//            arm.wristManualControl(operatorOI.right_stick_y.get() * ARM_MANUAL_SCALING);
//        }
//        else if(fourbar)
//            arm.wristFourbar();
//
        if(operatorOI.b.get())
            hand.setClawPos(1.0);
        else if(operatorOI.x.get())
            hand.setClawPos(0.5);

//        hand.setTurnyWristPos(0.5); //TODO: this functionality

//        if(operatorOI.right_bumper.get())
//            arm.setExtensionPos(0.7);
//        else if(operatorOI.left_bumper.get())
//            arm.setExtensionPos(0.0);

        if(operatorOI.left_trigger.get() > 0.0)
            arm.winchRobot();
        else if(operatorOI.right_trigger.get() > 0.0)
            arm.winchOpposite();
        else
            arm.stopWinch();

        arm.setPlaneLauncher(operatorOI.back.get());
    }

    private void setLightsColor() {
        if(!fieldOriented)
            lights.setLeftColour(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
        else if(splineToScoring || splineToIntake)
            lights.setLeftColour(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        else
            lights.setLeftColour(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);

        if(fourbar)
            lights.setLeftColour(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        else
            lights.setLeftColour(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
    }

    private void printTelemetry() {
        telemetry.addData("Drive Power", power);
        telemetry.addData("angle", angle);
        telemetry.addData("turn power", turn);

        double[] xy = drivetrain.getXY();
        telemetry.addData("x", xy[0]);
        telemetry.addData("y", xy[1]);

        telemetry.addData("heading", drivetrain.getFieldHeading());
        telemetry.addData("autoAligning", autoAlign);
        telemetry.addData("field oriented", fieldOriented);
//        telemetry.addData("arm angle", arm.getArmAngle());
//        telemetry.addData("wrist angle", arm.getWristAngle());
        telemetry.addData("fourbar???", fourbar);
        telemetry.addData("April Tag Detections", vision.getNumDetections() );

        telemetry.update();
    }
}
