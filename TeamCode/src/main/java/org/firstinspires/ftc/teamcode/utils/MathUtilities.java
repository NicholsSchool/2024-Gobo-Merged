package org.firstinspires.ftc.teamcode.utils;

/**
 * A Math Utilities Class for the Robot.
 */
public class MathUtilities implements Constants {
    /**
     * Adds two angles that are measured in degrees
     *
     * @param angle1 the first angle
     * @param angle2 the second angle
     *
     * @return the sum in the range [-180, 180)
     */
    public static double addAngles(double angle1, double angle2) {
        double sum = angle1 + angle2;
        while(sum >= 180.0)
            sum -= 360.0;
        while(sum < -180.0)
            sum += 360.0;
        return sum;
    }

    /**
     * Clips a number in the range [min, max]
     *
     * @param num the number to clip
     * @param min the minimum allowed value
     * @param max the minimum allowed value
     *
     * @return the number in the range
     */
    public static double clip(double num, double min, double max) {
        if(num > max)
            return max;
        return Math.max(num, min);
    }
}
