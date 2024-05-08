package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * The Intake Subsystem of the Robot
 */
public class Intake implements Constants {

    private final Servo leftServo;
    private final Servo rightServo;

    /**
     * Initializes pan servos (linear actuators)
     */
    public Intake(HardwareMap hwMap) {
        leftServo = hwMap.get(Servo.class, "leftDust");
        rightServo = hwMap.get(Servo.class, "rightDust");
    }
    /**
     * Raises or lowers the intake pan.
     *
     * @param isRaising whether to raise the pan
     */
    public void setPanPos(boolean isRaising) {
        leftServo.setPosition(isRaising ? INTAKE_UP_POSITION : INTAKE_DOWN_POSITION);
        rightServo.setPosition(isRaising ? INTAKE_UP_POSITION : INTAKE_DOWN_POSITION);
    }

    /**
     * Get the servo positions for telemetry
     *
     * @return the servo position (specifically the left one)
     */
    public double getPosition() {
        return leftServo.getPosition();
    }
}