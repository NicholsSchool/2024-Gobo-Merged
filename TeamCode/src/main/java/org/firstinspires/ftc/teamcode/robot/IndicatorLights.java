package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

//TODO: doesn't work

/**
 * The class defining the Indicator Lights on the robot.
 * A full list of REV blink codes can be found
 * <a href="https://first-tech-challenge.github.io/SkyStone/com/qualcomm/hardware/rev/RevBlinkinLedDriver.BlinkinPattern.html">here</a>.
 */
public class IndicatorLights {

    private final RevBlinkinLedDriver leftBlinkin;
    private final RevBlinkinLedDriver rightBlinkin;
    private final RevBlinkinLedDriver.BlinkinPattern defaultPattern;

    /**
     * Initializes the IndicatorLights subsystem
     *
     * @param hwMap the hardwareMap
     * @param isBlueAlliance whether we are blue alliance
     */
    public IndicatorLights(HardwareMap hwMap, boolean isBlueAlliance) {
        leftBlinkin = hwMap.get(RevBlinkinLedDriver.class,"leftBlinkin");
        rightBlinkin = hwMap.get(RevBlinkinLedDriver.class, "rightBlinkin");

        defaultPattern = isBlueAlliance ? BlinkinPattern.RAINBOW_OCEAN_PALETTE
                : BlinkinPattern.RAINBOW_LAVA_PALETTE;
        setColour(defaultPattern);
    }

    /** Sets both left and right LED strips to a certain colour pattern.
     *
     * @param pattern The pattern to set the LEDs to (BlinkinPattern)
     */
    public void setColour(BlinkinPattern pattern) {
        leftBlinkin.setPattern(pattern);
        rightBlinkin.setPattern(pattern);
    }

    /**
     * Sets left LED strip to a certain colour pattern
     *
     * @param pattern The pattern to set the LEDs to (BlinkinPattern)
     */
    public void setLeftColour(BlinkinPattern pattern) {
        leftBlinkin.setPattern(pattern);
    }

    /**
     * Sets right LED strip to a certain colour pattern
     *
     * @param pattern The pattern to set the LEDs to (BlinkinPattern)
     */
    public void setRightColour(BlinkinPattern pattern) {
        rightBlinkin.setPattern(pattern);
    }

    /**
     * Sets the default color, red or blue based on alliance.
     */
    public void setDefaultColor() {
        leftBlinkin.setPattern(defaultPattern);
        rightBlinkin.setPattern(defaultPattern);
    }
}