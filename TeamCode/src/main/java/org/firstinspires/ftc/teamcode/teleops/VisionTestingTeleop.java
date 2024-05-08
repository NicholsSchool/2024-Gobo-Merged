package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.teamcode.utils.Constants;

import java.util.HashMap;

/**
 * A teleop for testing robot Vision functionalities and accuracy
 */
@TeleOp(name="Vision Testing")
public class VisionTestingTeleop extends OpMode implements Constants
{
    private Vision vision;

    @Override
    public void init() {
        vision = new Vision(hardwareMap);
    }

    @Override
    public void loop() {
        double[] pose = vision.update();
        telemetry.addData("front detections", vision.getNumFrontDetections());
        telemetry.addData("back detections", vision.getNumBackDetections());
        telemetry.addData("total detections", vision.getNumDetections());
        telemetry.addData("X", pose != null ? pose[0] : "null");
        telemetry.addData("Y", pose != null ? pose[1] : "null");
        telemetry.addData("Theta", pose != null ? pose[2] : "null");

        HashMap<Integer, Double[]> headings = vision.getHeadings();
        telemetry.addData("ID: 1", headings.containsKey(1) ? headings.get(1) : "null");
        telemetry.addData("ID: 2", headings.containsKey(2) ? headings.get(2) : "null");
        telemetry.addData("ID: 3", headings.containsKey(3) ? headings.get(3) : "null");
        telemetry.addData("ID: 4", headings.containsKey(4) ? headings.get(4) : "null");
        telemetry.addData("ID: 5", headings.containsKey(5) ? headings.get(5) : "null");
        telemetry.addData("ID: 6", headings.containsKey(6) ? headings.get(6) : "null");
        telemetry.addData("ID: 7", headings.containsKey(7) ? headings.get(7) : "null");
        telemetry.addData("ID: 8", headings.containsKey(8) ? headings.get(8) : "null");
        telemetry.addData("ID: 9", headings.containsKey(9) ? headings.get(9) : "null");
        telemetry.addData("ID: 10", headings.containsKey(10) ? headings.get(10) : "null");
        telemetry.update();
    }
}
