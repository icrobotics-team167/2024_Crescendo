package frc.robot.abstraction.motors;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * An abstraction class for motors.
 */
public abstract class Motor {
    /**
     * The maximum number of times that the motor will attempt to apply
     * configurations,
     * if it takes more than this amount of tries assume there's something wrong and
     * abort.
     */
    public final int maximumRetries = 5;

    /**
     * Reset motor configuration back to factory defaults.
     */
    public abstract void factoryDefaults();

    /**
     * Clear sticky faults on the motor controller.
     */
    public abstract void clearStickyFaults();

    /**
     * Configure the integrated encoder in the motor. Sets the conversion factor for
     * position and velocity.
     * 
     * @param positionConversionFactor The conversion factor. The motor's position
     *                                 in rotations and velocity in rotations per
     *                                 second
     *                                 will be multiplied by this value.
     */
    public abstract void configureIntegratedEncoder(double positionConversionFactor);

    /**
     * Configure the control values for the closed loop PID controller. 0 is
     * disabled/off.
     * 
     * @param P Proportional value. If you're far away from your target, get there
     *          faster.
     * @param I Intergral value. If you've been away from your target for a while,
     *          increase motor power to push past it. Usually not needed.
     * @param D Derivative value. If you're close to your target, slow down.
     */
    public abstract void configrePID(double P, double I, double D);

    /**
     * Configure the values in which the PID wraps around. Usually used for angles.
     * 
     * @param minValue Minimum value of the PID. If set, the PID will consider
     *                 maxValue as the same point.
     * @param maxValue Maximum value of the PID. If set, the PID will consider
     *                 minValue as the same point.
     */
    public abstract void configurePIDWrapping(double minValue, double maxValue);

    /**
     * Configure whether the motor is in 
     * @param brake
     */
    public abstract void setMotorBrake(boolean brake);

    /**
     * Set current limits for the motor.
     * 
     * @param nominalVoltage    Nominal motor voltage. If the voltage being sent to
     *                          the motor falls below this number, motor speeds will
     *                          be adjusted to compensate.
     * @param primaryAmpLimit   Primary current limit. If the motor's current draw
     *                          amperage goes above this number, motor speeds will
     *                          be adjusted to keep current draw below the number.
     * @param secondaryAmpLimit Secondary current limit. If the motor's current draw
     *                          amperage goes above this number, the motor will be
     *                          temporarily shut down completely. Keep this number
     *                          above primary amp limit.
     */
    public abstract void setCurrentLimits(double nominalVoltage, int primaryAmpLimit, int secondaryAmpLimit);

    public abstract void setAbsoluteEncoder();

    /**
     * Set desired motor speed.
     * 
     * @param setPoint Motor speeds. Goes from +1 (100% speed forwards) to -1 (-100%
     *                 speeds backwards)
     */
    public abstract void set(double setPoint);

    /**
     * If the motor is being used as a swerve drive motor, set the target speed.
     * 
     * @param setPoint    Target velocity, in meters per second.
     * @param feedForward Feedforward in volt-meters per second.
     */
    public abstract void setDriveReference(double setPoint, double feedForward);

    /**
     * If the motor is being used as a swerve turn motor, set the target angle.
     * 
     * @param setPoint Target angle, as a Rotation2d object.
     * @param position Current angle, as a Rotation2d object.
     */
    public abstract void setTurnReference(Rotation2d setPoint, Rotation2d position);

}
