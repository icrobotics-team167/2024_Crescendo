package frc.robot.arm;

import frc.robot.abstraction.motors.AbstractMotor;
import frc.robot.helpers.Telemetry;
import frc.robot.helpers.Telemetry.Verbosity;
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

        motor.setPosition(0);
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
        Telemetry.sendNumber("Extension.position", getPosition(), Verbosity.HIGH);
        // Prevents outwards motion if it's already too far out.
        if (isTooFarOut() && speed > 0) {
            motor.stop();
            return;
        }
        // Prevents inwards motion if it's already too far out.
        if (isTooFarIn() && speed < 0) {
            motor.stop();
            return;
        }
        // If it passes both checks, actually run the motor.
        // The motor's movement direction is inverted
        motor.set(-speed);
    }

    /**
     * Gets if the arm is too far up.
     * 
     * @return If the arm is above its max point, configured in Constants.
     */
    public boolean isTooFarOut() {
        return Telemetry.sendBoolean("Extension.isTooFarOut", getPosition() >= Arm.Extension.EXTENSION_MAX,
                Verbosity.MEDIUM);
    }

    /**
     * Gets if the arm is too far in.
     * 
     * @return If the arm is retracted past its min point, configured in Constants.
     */
    public boolean isTooFarIn() {
        if (!retractSensor.get() || getPosition() <= Arm.Extension.EXTENSION_MIN) {
            motor.setPosition(0);
            return Telemetry.sendBoolean("Extension.isTooFarIn", true, Verbosity.MEDIUM);
        }
        return Telemetry.sendBoolean("Extension.isTooFarIn", false, Verbosity.MEDIUM);
    }

    /**
     * Gets the position of the extension, in inches.
     * 
     * @return The position in inches
     */
    public double getPosition() {
        return Arm.Extension.INITIAL_POSITION - motor.getPosition();
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
