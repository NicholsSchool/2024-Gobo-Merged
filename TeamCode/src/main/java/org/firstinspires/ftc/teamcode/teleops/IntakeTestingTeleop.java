package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * A teleop for testing Intake functionalities using
 * FTC Dashboard
 */
@Config
@TeleOp(name="[DASHBOARD] Intake Testing")
public class IntakeTestingTeleop extends OpMode implements Constants
{
    public Intake intake;
    public static boolean isRaising;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        intake.setPanPos(isRaising);
    }
}
