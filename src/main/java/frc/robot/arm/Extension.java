package frc.robot.arm;

import frc.robot.abstraction.motors.AbstractMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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
     * The sensor to detect minimum extension.
     */
    private DigitalInput retractSensor;

    /**
     * Creates a new Extension object.
     * 
     * @param motor The right motor on the extension mechanism.
     */
    public Extension(AbstractMotor motor, DigitalInput retractSensor) {
        this.motor = motor;
        this.retractSensor = retractSensor;

        motor.configureEncoder(getInchesPerRotation());
        if (retractSensor.get()) {
            DriverStation.reportError(
                    "Retraction switch is not activated on boot, turn off the robot and push arm all the way in.",
                    false);
        }
    }

    /**
     * Moves the extension mechanism.
     * 
     * @param speed The speed in which to move the extension. 1.0 is extend out full
     *              speed, -1.0 is retract full speed.
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
        if (!retractSensor.get() || motor.getPosition() <= Arm.Extension.EXTENSION_MIN) {
            motor.setPosition(0);
            return true;
        }
        return false;
    }

    /**
     * Calculate how many degrees that the extensions for 1 full rotation of the
     * motor.
     * 
     * @return Degrees per rotation.
     */
    private double getInchesPerRotation() {
        return Arm.Extension.EXTENSION_GEAR_RATIO;
    }
}
