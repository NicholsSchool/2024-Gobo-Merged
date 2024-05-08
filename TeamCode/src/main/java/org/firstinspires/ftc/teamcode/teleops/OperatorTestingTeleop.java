package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Hand;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.utils.Constants;

//TODO: add back pot functions

/**
 * A teleop for testing all Operator functionalities using
 * FTC Dashboard
 */
@Config
@TeleOp(name="[DASHBOARD] Full Operator Testing")
public class OperatorTestingTeleop extends OpMode implements Constants
{
    public Arm arm;
    public static double shoulderPower;
    public static boolean climb;
    public static boolean unClimb;
    public static double extensionPosition;
    public static boolean launchPlane;
    public static double desiredArmAngle = 180.0;
    public static boolean ARM_PID;
    public static double desiredWristAngle;
    public static boolean fourbar = true;
    public Intake intake;
    public static boolean isRaising;
    public Hand hand;
    public static double turnyWristPos = 0.5;
    public static double clawPos = 0.5;

    @Override
    public void init() {
        arm = new Arm(hardwareMap);
        hand = new Hand(hardwareMap, clawPos);
        intake = new Intake(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
//        if(fourbar)
//            arm.wristFourbar();
//        else
            arm.setWristPos(desiredWristAngle);

//        if(ARM_PID)
//            arm.armGoToPos(desiredArmAngle);
//        else
            arm.armManualControl(shoulderPower);

        if(climb)
            arm.winchRobot();
        else if(unClimb)
            arm.winchOpposite();
        else
            arm.stopWinch();

//        arm.setExtensionPos(extensionPosition);
        arm.setPlaneLauncher(launchPlane);

        intake.setPanPos(isRaising);

        hand.setClawPos(clawPos);
        hand.setTurnyWristPos(turnyWristPos);

//        telemetry.addData("arm angle", arm.getArmAngle() );
        telemetry.addData("desired arm angle", desiredArmAngle);
//        telemetry.addData("pot", arm.getPot());
        telemetry.addData("wrist pos", arm.getWristAngle());
        telemetry.addData("desired wrist pos", desiredWristAngle);
        telemetry.update();
    }
}
