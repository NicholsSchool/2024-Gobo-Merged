package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.MathUtilities;

@Autonomous(name="Blue Auto")
public class BlueAuto extends LinearOpMode implements Constants {
    private ElapsedTime timer = new ElapsedTime();
    private Drivetrain drivetrain;
    private Intake intake;
    private double[] xy = {36, -64.5};
    double distance = Math.sqrt(Math.pow(36 - xy[0], 2) + Math.pow(-32 - xy[1], 2) );

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap, BLUE_ALLIANCE, xy[0], xy[1], 90.0);
        intake = new Intake(hardwareMap);

        xy = new double[]{36, -64.5};

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        timer.reset();

        while(opModeIsActive()) {
            while(distance > SPLINE_ERROR && timer.time() < 3) {
                xy = drivetrain.getXY();
                distance = Math.sqrt(Math.pow(36 - xy[0], 2) +
                        Math.pow(-32 - xy[1], 2) );
                double power = distance >= SPLINE_ERROR ? MathUtilities.clip(SPLINE_P * distance
                        , -SPLINE_GOVERNOR, SPLINE_GOVERNOR) : 0.0;

                double angle = 90.0;
                drivetrain.setDesiredHeading(90.0);
                drivetrain.drive(power, angle, 0.0, true, true);
            }

            drivetrain.driveTest(0.0);

            timer.reset();

            while(timer.time() < 3) {
                drivetrain.driveTest(0.0);
                intake.setPanPos(false);
            }
            timer.reset();

            while(timer.time() < 3) {
                drivetrain.driveTest(0.0);
                intake.setPanPos(true);
            }
            stop();
//            distance = distance = Math.sqrt(Math.pow(36 - xy[0], 2) + Math.pow(0 - xy[1], 2) );
//
//
//            while(distance > SPLINE_ERROR && timer.time() < 10) {
//                xy = drivetrain.getXY();
//                distance = Math.sqrt(Math.pow(-48 - xy[0], 2) +
//                        Math.pow(-12 - xy[1], 2) );
//                double power = distance >= SPLINE_ERROR ? MathUtilities.clip(SPLINE_P * distance
//                        , -SPLINE_GOVERNOR, SPLINE_GOVERNOR) : 0.0;
//
//                double angle = 90.0;
//                drivetrain.setDesiredHeading(0.0);
//                drivetrain.drive(power, angle, 0.0, true, true);
//            }

            telemetry.addData("Status", "Run Time: " + timer);
            telemetry.update();
        }
    }
}
