package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * The Hand Subsystem
 */
public class Hand implements Constants {
    private final Servo turnyWrist;
    private final Servo leftClaw;
    private final Servo rightClaw;

    /**
     * Initializes the Hand object
     *
     * @param hwMap the hardwareMap
     */
    public Hand(HardwareMap hwMap, double clawStartingPos) {
        turnyWrist = hwMap.get(Servo.class, "turnyWrist");
        leftClaw = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");

        turnyWrist.scaleRange(0.0, MAX_TURNY_WRIST);
        leftClaw.scaleRange(0.0, LEFT_CLAW_OPEN);
        rightClaw.scaleRange(0.0, RIGHT_CLAW_OPEN);

        turnyWrist.setPosition(0.5);
        leftClaw.setPosition(clawStartingPos);
        rightClaw.setPosition(clawStartingPos);
    }

    /**
     * Moves the wrist manually. 0 is left, 0.5 is center, 1 is right.
     *
     * @param position the position to go to [min, max]
     */
    public void setTurnyWristPos(double position) {
        turnyWrist.setPosition(position);
    }

    /**
     * Moves the wrist manually
     *
     * @param position the position [0, 1] with 1 being fully open
     */
    public void setClawPos(double position) {
        leftClaw.setPosition(position);
        rightClaw.setPosition(position);
    }
}
