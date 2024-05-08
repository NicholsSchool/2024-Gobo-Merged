package org.firstinspires.ftc.teamcode.controller;

import androidx.annotation.NonNull;

/**
 * The class defining Buttons on a GameController
 */
public class Button {
    private boolean state;
    private boolean previousState;
    private boolean toggleState;

    /**
     * Creates a Button Object
     */
    public Button() {
        this.state = false;
        this.previousState = false;
        this.toggleState = false;
    }

    /**
     * Whether the button is pressed
     *
     * @return true if the button is pressed, false otherwise
     */
    public boolean get() {
        return state;
    }

    /**
     * The button's toggle value starts as false and is switched when the button is pressed.
     *
     * @return the button's toggle value
     */
    public boolean getToggleState() {
        return toggleState;
    }

    /**
     * Whether the button was just pressed
     *
     * @return true if the button was just pressed, false otherwise
     */
    public boolean wasJustPressed() {
        return state && !previousState;
    }

    /**
     * Updates the button's current and previous states
     *
     * @param newState the state to set the current state to
     */
    public void updateStates(boolean newState) {
        previousState = state;
        state = newState;
        toggleState = (state && !previousState) != toggleState;
    }

    /**
     * Return the current state, for telemetry
     *
     * @return the button's value as String
     */
    @NonNull
    public String toString() {
        return String.valueOf(get());
    }
}
