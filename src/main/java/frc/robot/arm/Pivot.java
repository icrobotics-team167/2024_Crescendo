package frc.robot.arm;

import frc.robot.abstraction.motors.AbstractMotor;
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
     * The left motor on the pivot.
     */
    private AbstractMotor followerMotor; // VSCode is saying that this is never used, ignore that.

    /**
     * Creates a new Pivot object.
     * 
     * @param leaderMotor   The right motor on the pivot mechanism.
     * @param followerMotor The left motor on the pivot mechanism.
     */
    public Pivot(AbstractMotor leaderMotor, AbstractMotor followerMotor) {
        this.leaderMotor = leaderMotor;
        this.followerMotor = followerMotor;

        leaderMotor.configureIntegratedEncoder(getDegreesPerRotation());
        followerMotor.configureInverted(true);
        followerMotor.configureFollow(leaderMotor);
    }

    /**
     * Moves the pivot mechanism.
     * 
     * @param speed The speed in which to move the pivot. 1.0 is pivot down full
     *              speed, -1.0 is pivot up full speed.
     */
    public void move(double speed) {
        if (isTooFarUp() && speed < 0) {
            leaderMotor.stop();
            return;
        }
        if (isTooFarDown() && speed > 0) {
            leaderMotor.stop();
            return;
        }
        leaderMotor.set(speed);
    }

    /**
     * Gets if the arm is too far up.
     * 
     * @return If the arm is above its max point, configured in Constants.
     */
    public boolean isTooFarUp() {
        return leaderMotor.getPosition() >= Arm.Pivot.PIVOT_MAX;
    }

    /**
     * Gets if the arm is too far down.
     * 
     * @return If the arm is below its min point, configured in Constants.
     */
    public boolean isTooFarDown() {
        return leaderMotor.getPosition() <= Arm.Pivot.PIVOT_MIN;
    }

    /**
     * Calculate how many degrees that the pivots for 1 full rotation of the motor.
     * 
     * @return Degrees per rotation.
     */
    private double getDegreesPerRotation() {
        return 360.0 / Arm.Pivot.PIVOT_GEAR_RATIO;
    }
}
