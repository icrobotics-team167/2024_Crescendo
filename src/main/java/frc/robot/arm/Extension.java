package frc.robot.arm;

import frc.robot.abstraction.motors.AbstractMotor;
import frc.robot.Constants.Robot.Arm;

/**
 * Class for the arm's extension.
 */
public class Extension {
    /**
     * The right motor on the extension.
     */
    private AbstractMotor motor;

    /**
     * Creates a new Extension object.
     * 
     * @param motor   The right motor on the extension mechanism.
     */
    public Extension(AbstractMotor motor) {
        this.motor = motor;

        motor.configureIntegratedEncoder(getInchesPerRotation());
    }

    /**
     * Moves the extension mechanism.
     * 
     * @param speed The speed in which to move the extension. 1.0 is extend out full speed, -1.0 is retract full speed.
     */
    public void move(double speed) {
        if (isTooFarOut() && speed < 0) {
            motor.stop();
            return;
        }
        if (isTooFarIn() && speed > 0) {
            motor.stop();
            return;
        }
        motor.set(speed);
    }

    /**
     * Gets if the arm is too far up.
     * 
     * @return If the arm is above its max point, configured in Constants.
     */
    public boolean isTooFarOut() {
        return motor.getPosition() >= Arm.Extension.EXTENSION_MAX;
    }

    /**
     * Gets if the arm is too far down.
     * 
     * @return If the arm is below its min point, configured in Constants.
     */
    public boolean isTooFarIn() {
        return motor.getPosition() <= Arm.Extension.EXTENSION_MIN;
    }

    /**
     * Calculate how many degrees that the extensions for 1 full rotation of the motor.
     * 
     * @return Degrees per rotation.
     */
    private double getInchesPerRotation() {
        return Arm.Extension.EXTENSION_GEAR_RATIO;
    }
}
