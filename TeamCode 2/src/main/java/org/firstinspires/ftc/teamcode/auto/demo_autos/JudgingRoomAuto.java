package org.firstinspires.ftc.teamcode.auto.demo_autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.constants.SplineConstants;
import org.firstinspires.ftc.teamcode.math.Angles;
import org.firstinspires.ftc.teamcode.math.ParabolicSpline;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.RobotPose;
import org.firstinspires.ftc.teamcode.math.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Auto for Demo-ing Spline in the judging room
 */
@Autonomous(name="Judging Room", group="Demo")
public class JudgingRoomAuto extends LinearOpMode implements RobotConstants, SplineConstants {
    @Override
    public void runOpMode() {
        ElapsedTime timer;
        Drivetrain drivetrain;
        ParabolicSpline spline;

        waitForStart();
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        drivetrain = new Drivetrain(hardwareMap, 0.0, 0.0, Angles.PI_OVER_TWO);
        drivetrain.setFloat();
        spline = new ParabolicSpline(drivetrain, IS_BLUE_ALLIANCE);

        double distance = SPLINE_ERROR;
        Point destination = new Point(72.0, 72.0);

        while (distance >= SPLINE_ERROR) {
            drivetrain.update();
            RobotPose pose = drivetrain.getRobotPose();

            Vector input = spline.vectorToVertex(pose.toPoint(), destination, true);

            distance = Math.hypot(72.0 - pose.x, 72.0 - pose.y);
            double powerRatio = (distance >= SPLINE_ERROR ? SPLINE_P * distance : 0.0) / Math.hypot(input.x, input.y);

            input.x *= powerRatio;
            input.y *= powerRatio;

            drivetrain.drive(input, 0.1, false, true);
        }

        drivetrain.drive(new Vector(0.0, 0.0), 0.0, false, false);
        terminateOpModeNow();
    }
}