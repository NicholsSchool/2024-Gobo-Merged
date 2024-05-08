package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.BezierSpline;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Hand;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.firstinspires.ftc.teamcode.subsystems.PropDetector;

/**
 * Robot Container for the Autonomous Period
 */
public class AutonomousRobot {
    private final PropDetector propDetector;
    private final Arm arm;
    private final Drivetrain drivetrain;
    private final Hand hand;
    private final Lights lights;
    private final BezierSpline path;
    private Point[] splinePoints;

    /**
     * @param hwMap the hardware map
     * @param x the initial x
     * @param y the initial y
     * @param angle the initial angle
     * @param isBlue whether we are blue alliance
     */
    public AutonomousRobot(HardwareMap hwMap, double x, double y, double angle, boolean isBlue) {
        propDetector = new PropDetector(hwMap);
        arm = new Arm(hwMap, 0, 0);
        drivetrain = new Drivetrain(hwMap, x, y, angle);
        hand = new Hand(hwMap);
        lights = new Lights(hwMap, isBlue);
        splinePoints = isBlue ? new Point[] {new Point (35.2, -34.9), new Point(67.1, -83.5),
                new Point(82.9, 56.0), new Point(-44.5, -36.3)} : new Point[]
                {new Point (35.2, 34.9), new Point(67.1, 83.5),
                new Point(82.9, -56.0), new Point(-44.5, 36.3)};
        path = new BezierSpline(drivetrain, splinePoints, 10, 100);
    }

    public void update(){
        drivetrain.update();
    }

}
