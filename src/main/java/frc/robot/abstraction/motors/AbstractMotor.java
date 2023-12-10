package frc.robot.abstraction.motors;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.abstraction.encoders.AbstractAbsoluteEncoder;

/**
 * An abstraction class for motors.
 */
public abstract class AbstractMotor {
    /**
     * The maximum number of times that the motor will attempt to apply
     * configurations,
     * if it takes more than this amount of tries assume there's something wrong and
     * abort.
     */
    public final int maximumRetries = 5;

    /**
     * Reset motor configuration back to factory defaults. Can only be run once.
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
    public abstract void configurePID(double P, double I, double D);

    /**
     * Configure whether or not the PIDs should wrap around 180/-180, considering them the same point.
     * 
     * @param wrapPID Whether or not to wrap the PID or not.
     */
    public abstract void configurePIDWrapping(boolean wrapPID);

    /**
     * Configure whether the motor is in brake mode or coast mode.
     * 
     * @param brake If the motor is in brake mode or not. Coast mode if false.
     */
    public abstract void configureMotorBrake(boolean brake);

    /**
     * Configure whether or not the motor output is inverted or not.
     * 
     * @param inverted If the motor is inverted or not. True is inverted.
     */
    public abstract void configureInverted(boolean inverted);

    /**
     * Set current limits for the motor.
     * 
     * @param nominalVoltage    Nominal motor voltage. If the voltage being sent to
     *                          the motor falls below this number, motor speeds will
     *                          be adjusted to compensate.
     * @param primaryAmpLimit   Primary current limit. If the motor's current draw
     *                          amperage goes above this number, motor speeds will
     *                          be adjusted to keep current draw below the number.
     *                          Can cause issues with conflicting compensations
     *                          making the motor stutter if set too low.
     * @param secondaryAmpLimit Secondary current limit. If the motor's current draw
     *                          amperage goes above this number, the motor will be
     *                          temporarily shut down completely. Keep this number
     *                          above primary amp limit.
     */
    public abstract void configureCurrentLimits(double nominalVoltage, int primaryAmpLimit, int secondaryAmpLimit);

    /**
     * Set the absolute encoder to be used by the motor.
     * 
     * @param absoluteEncoder The encoder to be used.
     */
    public abstract void configureAbsoluteEncoder(AbstractAbsoluteEncoder absoluteEncoder);

    /**
     * Set how long the motor can take to go from 0 to max power.
     * 
     * @param rampRate Time, in seconds, that it takes to accelerate.
     */
    public abstract void configureRampRate(double rampRate);

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
     */
    public abstract void setTurnReference(Rotation2d setPoint);

    /**
     * Sets the motor encoder's position.
     * 
     * @param position The new position.
     */
    public abstract void setPosition(double position);

    /**
     * Get the motor object.
     * 
     * @return The motor object.
     */
    public abstract Object getMotor();

    /**
     * Gets whether or not an absolute encoder is connected to the motor.
     * 
     * @return If an absolute encoder is connected or not.
     */
    public abstract boolean isAttachedAbsoluteEncoder();

    /**
     * Gets the velocity measured by the encoder.
     * 
     * @return Velocity. Is in rotations per second by default but can be changed by
     *         using configureIntegratedEncoder().
     */
    public abstract double getVelocity();

    /**
     * Gets the position of the integrated encoder.
     * 
     * @return Position. Is in rotations by default but can be changed by using
     *         configureIntegratedEncoder().
     */
    public abstract double getPosition();

    /**
     * Gets the nominal voltage of the motor. Configured in Constants.Robot.Motors.
     * 
     * @return Nominal voltage, in volts.
     */
    public abstract double getNominalVoltage();

    /**
     * Gets the primary current limit of the motor. Configured in
     * Constants.Robot.Motors.
     * 
     * @return Primary current limit, in amps.
     */
    public abstract int getPrimaryCurrentLimit();

    /**
     * Gets the secondary current limit of the motor. Configured in
     * Constants.Robot.Motors.
     * 
     * @return Secondary current limit, in amps.
     */
    public abstract int getSecondaryCurrentLimit();

    /**
     * Gets the max velocity of the motor. Configured in Constants.Robot.Motors.
     * 
     * @return Max velocity, in RPM.
     */
    public abstract double getMaxRPM();
}