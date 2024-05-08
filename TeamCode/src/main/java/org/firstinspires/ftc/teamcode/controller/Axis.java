package org.firstinspires.ftc.teamcode.controller;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * The class defining Axes on a GameController
 */
public class Axis implements Constants {
    private final double deadBand;
    private int numLoopsAtZero;
    private double value;

    /**
     * Creates an Axis Object with the default deadBand defined in Constants
     */
    public Axis() {
        this(DEFAULT_DEADBAND);
    }

    /**
     * Creates an Axis Object with a specified deadBand
     *
     * @param deadBand the deadBand
     */
    public Axis(double deadBand) {
        this.value = 0.0;
        this.deadBand = deadBand;
        this.numLoopsAtZero = 0;
    }

    /**
     * The current value of the axis
     *
     * @return the value in the range [-1, 1]
     */
    public double get() {
        return value;
    }

    /**
     * Whether the Axis has been zero for
     * a specified number of teleop loop() iterations
     *
     * @return true iff the Axis has been zero for enough code loops
     */
    public boolean wasZeroLongEnough() {
        return numLoopsAtZero >= LOOPS_TO_WAIT;
    }

    /**
     * Updates the axis's current and previous values
     *
     * @param newValue the value to set the current value to
     */
    public void updateStates(double newValue) {
        value = applyDeadBand(newValue);
        numLoopsAtZero = value == 0 ? numLoopsAtZero + 1 : 0;
    }

    private double applyDeadBand(double value) {
        return Math.abs(value) >= deadBand ? value : 0.0;
    }

    /**
     * Return the current value, for telemetry
     *
     * @return the axis's value as String
     */
    @NonNull
    public String toString() {
        return String.valueOf(get());
    }
}
