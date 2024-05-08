package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.MathUtilities;

/**
 * The Class defining a GameController
 */
public class GameController {
    private final Gamepad gamepad;
    public Button y;
    public Button x;
    public Button b;
    public Button a;
    public Button dpad_up;
    public Button dpad_down;
    public Button dpad_left;
    public Button dpad_right;
    public Button left_bumper;
    public Button right_bumper;
    public Button start;
    public Button back;
    public Button right_stick_button;
    public Button left_stick_button;
    public Axis left_stick_x;
    public Axis left_stick_y;
    public Axis right_stick_x;
    public Axis right_stick_y;
    public Axis right_trigger;
    public Axis left_trigger;

    /**
     * Constructs the GameController with a Gamepad
     *
     * @param gamepad the gamepad to monitor
     */
    public GameController(Gamepad gamepad) {
        this.gamepad = gamepad;

        y = new Button();
        x = new Button();
        b = new Button();
        a = new Button();
        dpad_up = new Button();
        dpad_down = new Button();
        dpad_left = new Button();
        dpad_right = new Button();
        left_bumper = new Button();
        right_bumper = new Button();
        start = new Button();
        back = new Button();
        right_stick_button = new Button();
        left_stick_button = new Button();

        left_stick_x = new Axis();
        left_stick_y = new Axis();
        right_stick_x = new Axis();
        right_stick_y = new Axis();
        right_trigger = new Axis();
        left_trigger = new Axis();
    }

    /**
     * The R from the (R, theta) of the Left Joystick
     *
     * @return the radius in the range [-1, 1]
     */
    public double leftStickRadius() {
        double x = left_stick_x.get();
        double y = left_stick_y.get();
        return MathUtilities.clip(Math.sqrt(x * x + y * y), -1.0, 1.0);
    }

    /**
     * The theta from the (R, theta) of the Left Joystick
     *
     * @param isBlue whether we are on Blue Alliance
     * @return the theta in the range [-180, 180)
     */
    public double leftStickTheta(boolean isBlue) {
        if (isBlue)
            return MathUtilities.addAngles(Math.toDegrees(Math.atan2(left_stick_y.get(), left_stick_x.get())), 0.0);
        else
            return MathUtilities.addAngles(Math.toDegrees(Math.atan2(left_stick_y.get(), left_stick_x.get())), -180.0);
    }

    /**
     * updates the GameController fields, call at the start of each loop() cycle
     */
    public void updateValues() {
        y.updateStates(gamepad.y);
        x.updateStates(gamepad.x);
        b.updateStates(gamepad.b);
        a.updateStates(gamepad.a);
        dpad_up.updateStates(gamepad.dpad_up);
        dpad_down.updateStates(gamepad.dpad_down);
        dpad_left.updateStates(gamepad.dpad_left);
        dpad_right.updateStates(gamepad.dpad_right);
        left_bumper.updateStates(gamepad.left_bumper);
        right_bumper.updateStates(gamepad.right_bumper);
        start.updateStates(gamepad.start);
        back.updateStates(gamepad.back);
        right_stick_button.updateStates(gamepad.right_stick_button);
        left_stick_button.updateStates(gamepad.left_stick_button);

        left_stick_x.updateStates(gamepad.left_stick_x);
        left_stick_y.updateStates(-gamepad.left_stick_y);
        right_stick_x.updateStates(gamepad.right_stick_x);
        right_stick_y.updateStates(-gamepad.right_stick_y);
        right_trigger.updateStates(gamepad.right_trigger);
        left_trigger.updateStates(gamepad.left_trigger);
    }
}
