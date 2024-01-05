package frc.robot.arm;

import frc.robot.abstraction.motors.AbstractMotor;
import frc.robot.helpers.Telemetry;
import frc.robot.helpers.Telemetry.Verbosity;
import frc.robot.Constants.Robot.Arm;

/**
 * Class for the arm's pivot.
 */
public class Pivot {
    /**
     * The right motor on the pivot.
     */
    private AbstractMotor leaderMotor;

    /**
     * Creates a new Pivot object.
     * 
     * @param leaderMotor   The right motor on the pivot mechanism.
     * @param followerMotor The left motor on the pivot mechanism.
     */
    public Pivot(AbstractMotor leaderMotor, AbstractMotor followerMotor) {
        // Make sure the motors are in brake mode.
        leaderMotor.configureMotorBrake(true);
        followerMotor.configureMotorBrake(true);

        // Configure encoders
        leaderMotor.setPosition(0);
        leaderMotor.configureEncoder(getDegreesPerRotation());

        // Configure follower motor.
        followerMotor.configureFollow(leaderMotor, true);

        this.leaderMotor = leaderMotor;
    }

    /**
     * Moves the pivot mechanism.
     * 
     * @param speed The speed in which to move the pivot. 1.0 is pivot down full
     *              speed, -1.0 is pivot up full speed.
     */
    public void move(double speed) {
        Telemetry.sendNumber("Pivot.position", getPosition(), Verbosity.HIGH);
        // Prevent upwards motion if it's already too far up.
        if (isTooFarUp() && speed < 0) {
            leaderMotor.stop();
            return;
        }
        // Prevents downwards motion if it's already too far down.
        if (isTooFarDown() && speed > 0) {
            leaderMotor.stop();
            return;
        }
        // If it passes both checks, actually move the pivot.
        leaderMotor.set(speed);
    }

    /**
     * Gets if the arm is too far up.
     * 
     * @return If the arm is above its max point, configured in Constants.
     */
    public boolean isTooFarUp() {
        return Telemetry.sendBoolean("Pivot.isTooFarUp", getPosition() >= Arm.Pivot.PIVOT_MAX, Verbosity.MEDIUM);
    }

    /**
     * Gets if the arm is too far down.
     * 
     * @return If the arm is below its min point, configured in Constants.
     */
    public boolean isTooFarDown() {
        return Telemetry.sendBoolean("Pivot.isTooFarDown", getPosition() <= Arm.Pivot.PIVOT_MIN, Verbosity.MEDIUM);
    }

    /**
     * Gets the position of the pivot, in degrees.
     * 
     * @return The position in degrees
     */
    public double getPosition() {
        return Arm.Pivot.INITIAL_POSITION - leaderMotor.getPosition();
    }

    /**
     * Calculate how many degrees that it pivots for 1 full rotation of the motor.
     * 
     * @return Degrees per rotation.
     */
    private double getDegreesPerRotation() {
        return 360.0 / Arm.Pivot.PIVOT_GEAR_RATIO;
    }
}
