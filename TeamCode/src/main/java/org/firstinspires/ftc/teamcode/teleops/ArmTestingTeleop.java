package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.utils.Constants;

//TODO: add back pot functions

/**
 * A teleop for testing Arm functionalities using
 * FTC Dashboard
 */
@Config
@TeleOp(name="[DASHBOARD] Arm Testing")
public class ArmTestingTeleop extends OpMode implements Constants
{
    public Arm arm;
    public static double shoulderPower;
    public static boolean climb;
    public static boolean unClimb;
    public static double extensionPosition;
    public static boolean launchPlane;
    public static double desiredArmAngle;
    public static boolean ARM_PID;
    public static int desiredWristAngle;
    public static boolean fourbar;

    @Override
    public void init() {
        arm = new Arm(hardwareMap);
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

//        telemetry.addData("arm angle", arm.getArmAngle() );
        telemetry.addData("desired arm angle", desiredArmAngle);
//        telemetry.addData("pot", arm.getPot());
        telemetry.addData("wrist pos", arm.getWristAngle());
        telemetry.addData("desired wrist pos", desiredWristAngle);
        telemetry.update();
    }
}
