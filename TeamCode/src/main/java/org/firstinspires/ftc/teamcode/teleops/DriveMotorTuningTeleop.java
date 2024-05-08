package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * A teleop for tuning drive motors using
 * FTC Dashboard
 */
@Config
@TeleOp(name="[DASHBOARD] Drive Motor Tuning")
public class DriveMotorTuningTeleop extends OpMode implements Constants
{
    private final ElapsedTime runtime = new ElapsedTime();
    public Drivetrain drivetrain;
//    public static double lbp = BACK_LEFT_P;
//    public static double lbi = BACK_LEFT_I;
//    public static double lbd = BACK_LEFT_D;
//    public static double lbf = BACK_LEFT_F;
//    public static double rbp = BACK_RIGHT_P;
//    public static double rbi = BACK_RIGHT_I;
//    public static double rbd = BACK_RIGHT_D;
//    public static double rbf = BACK_RIGHT_F;
//    public static double lfp = FRONT_LEFT_P;
//    public static double lfi = FRONT_LEFT_I;
//    public static double lfd = FRONT_LEFT_D;
//    public static double lff = FRONT_LEFT_F;
//    public static double rfp = FRONT_RIGHT_P;
//    public static double rfi = FRONT_RIGHT_I;
//    public static double rfd = FRONT_RIGHT_D;
//    public static double rff = FRONT_RIGHT_F;
    public static double p = 10;
    public static double i = 0;
    public static double d = 1;
    public static double f = 14;
    public static double targetMotorVelocity;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, true, 0.0, 0.0, 90.0);
//        drivetrain.backLeft.setVelocityPIDFCoefficients(lbp, lbi, lbd, lbf);
//        drivetrain.backRight.setVelocityPIDFCoefficients(rbp, rbi,rbd, rbf);
//        drivetrain.frontLeft.setVelocityPIDFCoefficients(lfp, lfi, lfd, lff);
//        drivetrain.frontRight.setVelocityPIDFCoefficients(rfp, rfi, rfd, rff);
        drivetrain.backLeft.setVelocityPIDFCoefficients(p, i, d, f);
        drivetrain.backRight.setVelocityPIDFCoefficients(p, i,d, f);
        drivetrain.frontLeft.setVelocityPIDFCoefficients(p, i, d, f);
        drivetrain.frontRight.setVelocityPIDFCoefficients(p, i, d, f);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if(runtime.time() > 5.0) {
            runtime.reset();
            targetMotorVelocity *= -1;
        }

        drivetrain.backLeft.setVelocityPIDFCoefficients(p, i, d, f);
        drivetrain.backRight.setVelocityPIDFCoefficients(p, i,d, f);
        drivetrain.frontLeft.setVelocityPIDFCoefficients(p, i, d, f);
        drivetrain.frontRight.setVelocityPIDFCoefficients(p, i, d, f);

        drivetrain.driveTest(targetMotorVelocity / 2800.0);

        double backLeftVel = drivetrain.backLeft.getVelocity();
        double backRightVel = drivetrain.backRight.getVelocity();
        double frontLeftVel = drivetrain.frontLeft.getVelocity();
        double frontRightVel = drivetrain.frontRight.getVelocity();

        telemetry.addData("back left vel", backLeftVel);
        telemetry.addData("back right vel", backRightVel);
        telemetry.addData("front left vel", frontLeftVel);
        telemetry.addData("front right vel", frontRightVel);
        telemetry.addData("target vel", targetMotorVelocity);
        telemetry.update();
    }
}
