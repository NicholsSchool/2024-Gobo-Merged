package org.firstinspires.ftc.teamcode.auto.competition_autos;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.constants.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.constants.*;
import org.firstinspires.ftc.teamcode.math.*;

import java.util.List;

/**
 * Sample Auto to Copy Paste Edit with
 */
@Autonomous(name = "Red Auto")
public class RedAuto extends LinearOpMode implements DrivetrainConstants, ArmConstants {
    /**
     * Runs the Auto routine
     */
    @Override
    public void runOpMode() {
        ElapsedTime waitTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        ElapsedTime sampleTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        Drivetrain drivetrain = new Drivetrain(hardwareMap, 36, 55.25, -Math.PI / 2);
        Arm arm = new Arm(hardwareMap, 0, 0);
        Hand hand = new Hand(hardwareMap);
        PropDetector propDetector = new PropDetector(hardwareMap);
        List<Recognition> recs;

        Lights lights = new Lights(hardwareMap, false);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GRAY);


        double[][] points1 = new double[][]{{35.8, 62.8}, {35.8, 62.8}, {35.4, 33.6}, {35.4, 33.6}};
        double[][] points2 = new double[][]{{36.0, 38.0}, {47.4, 69.6}, {76.8, 22.1}, {38.8, 4.2}};
        double[][] points4 = new double[][]{{-47.0, 37.1}, {-47.8, 29.5}, {-39.6, 8.4}, {-64.4, 10.8}};

        Spline spline1 = new Spline (points1, 20, drivetrain, 100);
        Spline spline2 = new Spline(points2, 10, drivetrain, 250);
        Spline spline4 = new Spline(points4, 10, drivetrain, 250);
        spline1.update();
        spline2.update();
        spline4.update();

        PropDetector.PropLocation propPosition = null;
        while (opModeInInit()) {
            propPosition = propDetector.getPropPosition();
            telemetry.addData("prop position", propPosition);
            telemetry.update();
        }


        waitForStart();


        propDetector.close();
        AprilTagVision vision = new AprilTagVision(hardwareMap);


        double purplePixelAngle = Math.PI / 2;
        double swipeHeading = Math.PI / 2;

        double endpoint = 34.6;

        if (propPosition != null) {
            if (propPosition == PropDetector.PropLocation.CENTER) {
                purplePixelAngle = 0;
                swipeHeading = -Math.PI / 2;
                endpoint = 29.4;
            } else if (propPosition == PropDetector.PropLocation.LEFT) {
                purplePixelAngle = 0;
                swipeHeading = - 0.3;
                endpoint = 22.0;
            }else if(propPosition == PropDetector.PropLocation.RIGHT){
                purplePixelAngle = Math.PI;
                swipeHeading = Math.PI + 0.8;
                endpoint = 35.4;
            }
        } else {
            purplePixelAngle = 0;
            swipeHeading = Math.PI / 2;
        }

        double[][] points3 = new double[][]{{38.8, 4.2}, {-22.4, 6.7}, {-45.7, -7.8}, {-45, endpoint}};
        Spline spline3 = new Spline(points3, 20, drivetrain, 200);
        spline3.update();

        while (spline1.desiredT() < 0.98) {
            spline1.update();
            double[] robotPose = new double[]{drivetrain.getRobotPose().x, drivetrain.getRobotPose().y};

            double jamesSplineError = Math.hypot(robotPose[0] - points2[3][0], robotPose[1] - points2[3][1]);

            arm.setTargetArmPosition(600);
            arm.armToPosition();

            telemetry.addData("wrist", arm.getWristPosition());
            telemetry.update();

            double turn = 0;
            boolean autoAlign = true;

            double desiredT = spline2.desiredT();

            drivetrain.drive(new Vector(Math.cos(spline1.angle()), Math.sin(spline1.angle())), turn, autoAlign, true);
            if (sampleTime.time() > 10) {
                spline1.update();
                sampleTime.reset();
            }

        }

        waitTime.reset();
        while (waitTime.time() < 2) {
            drivetrain.update();
            drivetrain.drive(new Vector(0, 0), 0, true, false);
            drivetrain.setTargetHeading(purplePixelAngle);

            arm.setTargetWristPosition(4400);
            arm.wristToPosition();

        }

        waitTime.reset();
        while(waitTime.time() < 1){

            arm.setTargetArmPosition(-500);
            arm.armToPosition();

            arm.setTargetWristPosition(4200);
            arm.wristToPosition();

            drivetrain.update();
            drivetrain.drive(new Vector(0, 0), 0, true, false);
        }

        waitTime.reset();
        while(waitTime.time() < 1){
            drivetrain.setTargetHeading(swipeHeading);

            arm.setTargetArmPosition(-500);
            arm.armToPosition();

            arm.setTargetWristPosition(4600);
            arm.wristToPosition();

            drivetrain.update();
            drivetrain.drive(new Vector(0, 0), 0, true, false);
        }


        hand.toggleRight();
        waitTime.reset();
        while(waitTime.time() < 0.5 && propPosition == PropDetector.PropLocation.RIGHT){
            drivetrain.drive(new Vector(0.2, 0), 0, true, false);
        }

        arm.offsetEncoders();
        arm.setTargetArmPosition(Math.PI);
        arm.setTargetWristPosition(0);

        while (spline2.desiredT() < 0.98) {
            spline2.update();
            double[] robotPose = new double[]{drivetrain.getRobotPose().x, drivetrain.getRobotPose().y};

            arm.setTargetWristPosition(1000);
            arm.wristToPosition();

            if(spline2.desiredT() > 0.2) {
                arm.setTargetArmPosition(0);
                arm.armToPosition();
            }

            double jamesSplineError = Math.hypot(robotPose[0] - points2[3][0], robotPose[1] - points2[3][1]);
            double desiredT = spline2.desiredT();

            drivetrain.setTargetHeading(spline2.desiredT() < 0.7 ? -Math.PI / 2 : 0);
            double turn = 0;
            boolean autoAlign = true;


            drivetrain.drive(new Vector(Math.cos(spline2.angle()), Math.sin(spline2.angle())), turn, autoAlign, true);
            if (sampleTime.time() > 20) {
                spline2.update();
                sampleTime.reset();
            }
        }

        hand.toggleRight();

        while (spline3.desiredT() < 0.98) {
            spline3.update();
            double[] robotPose = new double[]{drivetrain.getRobotPose().x, drivetrain.getRobotPose().y};

            double jamesSplineError = Math.hypot(robotPose[0] - points2[3][0], robotPose[1] - points2[3][1]);
            double desiredT = spline3.desiredT();

            arm.setTargetArmPosition(0);
            arm.armToPosition();

            drivetrain.setTargetHeading(desiredT < 0.9 ? -0.1 : Math.PI);
            double turn = 0;
            boolean autoAlign = true;

            drivetrain.drive(new Vector(Math.cos(spline3.angle()), Math.sin(spline3.angle())), turn, autoAlign, true);
            if (sampleTime.time() > 20) {
                spline3.update();
                sampleTime.reset();
            }
        }

        waitTime.reset();
        while(waitTime.time() < 2){
            arm.setTargetArmPosition(625);
            arm.armToPosition();
            arm.virtualFourbar();
            drivetrain.update();
            drivetrain.drive(new Vector(0, 0), 0, true, false);

        }

        waitTime.reset();
        while(waitTime.time() < 0.5){
            drivetrain.update();
            drivetrain.drive(new Vector(-0.3, 0), 0, true, true);
        }

        hand.toggleLeft();

        waitTime.reset();
        while(waitTime.time() < 0.6){
            drivetrain.update();
            drivetrain.drive(new Vector(0.3, 0), 0, true, true);
        }

        waitTime.reset();
        while(waitTime.time() < 3){

            drivetrain.setTargetHeading(-Math.PI / 2);

            if(waitTime.time() > 0.3) {
                arm.setTargetArmPosition(0);
                arm.armToPosition();
            }
            arm.setTargetWristPosition(-50.0);
            arm.wristToPosition();
            drivetrain.update();
            drivetrain.drive(new Vector(0, 0), 0, true, false);

        }
        drivetrain.drive(new Vector(0, 0), 0, false, false);

        terminateOpModeNow();
    }
}