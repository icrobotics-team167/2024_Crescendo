package frc.robot.arm;

import frc.robot.abstraction.motors.AbstractMotor;

/**
 * Class for the arm's claw.
 */
public class Claw {
    /**
     * The claw motor.
     */
    private AbstractMotor motor;

    /**
     * Creates a new Claw object.
     * 
     * @param motor   The motor on the claw mechanism.
     */
    public Claw(AbstractMotor motor) {
        this.motor = motor;
    }

    /**
     * Moves the claw mechanism.
     * 
     * @param speed The speed in which to move the claw. 1.0 is extend out full speed, -1.0 is retract full speed.
     */
    public void move(double speed) {
        motor.set(speed);
    }
}
