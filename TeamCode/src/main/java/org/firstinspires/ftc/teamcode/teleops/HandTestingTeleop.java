package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Hand;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * A teleop for testing Hand functionalities using
 * FTC Dashboard
 */
@Config
@TeleOp(name="[DASHBOARD] Hand Testing")
public class HandTestingTeleop extends OpMode implements Constants
{
    public Hand hand;
    public static double turnyWristPos;
    public static double clawPos;

    @Override
    public void init() {
        hand = new Hand(hardwareMap, clawPos);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turnyWristPos = 0.5;
    }

    @Override
    public void loop() {
        hand.setClawPos(clawPos);
        hand.setTurnyWristPos(turnyWristPos);
    }
}
