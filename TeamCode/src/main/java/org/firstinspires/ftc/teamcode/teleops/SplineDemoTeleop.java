package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.MathUtilities;

/**
 * Spline Demo Teleop
 */
@TeleOp(name = "Spline Demo")
public class SplineDemoTeleop extends OpMode implements Constants {
    private Drivetrain drivetrain;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, true, 0.0, 0.0, 90.0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drivetrain.updateWithOdometry();

        double[] xy = drivetrain.getXY();

        double distance = Math.sqrt(Math.pow(100 - xy[0], 2) +
                Math.pow(100 - xy[1], 2) );
        double power = distance >= SPLINE_ERROR ? MathUtilities.clip(SPLINE_P * distance
                , -SPLINE_GOVERNOR, SPLINE_GOVERNOR) : 0.0;

        double angle = drivetrain.angleToVertex(100, 100, true);

        drivetrain.drive(power, angle, 0.1, false, true);
    }
}
