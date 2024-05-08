package org.firstinspires.ftc.teamcode.auto.sample_autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Sample Auto
 */
@Autonomous(name="TemplateAuto", group="Template")
public class SampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        ElapsedTime timer;

        waitForStart();
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        terminateOpModeNow();
    }
}