package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.MathUtils;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Moves the arm to a specified position.
 */
public class MoveArmTo extends Command {
    /**
     * The motor power multiplier for pivot motion. Basically a P value in a PID
     * controller.
     * 
     * <p>
     * Measured in motor power per inch of error, ex. a value of 0.5 = 50% power at
     * 1 degree error
     */
    private final double PIVOT_POWER_MULT = 0.25;
    /**
     * The motor power multiplier for extension motion. Basically a P value in a PID
     * controller.
     * 
     * <p>
     * Measured in motor power per inch of error, ex. a value of 0.5 = 50% power at
     * 1 inch error
     */
    private final double EXTENSION_POWER_MULT = 0.25;
    /**
     * The value in which, if the abs of the pivot error is less than this, we call
     * it "good enough" and stop the motor. Momentum will probably get us the rest
     * of the way there anyways.
     */
    private final double PIVOT_TOLERANCE = 1;
    /**
     * The value in which, if the abs of the pivot error is less than this, we call
     * it "good enough" and stop the motor. Momentum will probably get us the rest
     * of the way there anyways.
     */
    private final double EXTENSION_TOLERANCE = 1;

    /**
     * The arm subsystem.
     */
    private ArmSubsystem arm;
    /**
     * The target position.
     */
    private ArmPosition targetPosition;

    /**
     * The pivot error, or the difference between the target angle and the current
     * angle.
     */
    private double pivotError;
    /**
     * The extension error, or the difference between the target extension and the
     * current extension.
     */
    private double extensionError;

    /**
     * Constructs a new MoveArmTo commmand.
     * 
     * @param arm            The arm subsystem.
     * @param targetPosition The target position of the arm.
     */
    public MoveArmTo(ArmSubsystem arm, ArmPosition targetPosition) {
        this.arm = arm;
        this.targetPosition = targetPosition;
        addRequirements(this.arm);
        setName("Move Arm");
    }

    @Override
    public void execute() {
        // This method runs every robot tick until the command is stopped.
        // Calculate error
        calculateError();
        // Calculate motor power from error
        double pivotOutput = MathUtils.applyDeadzone(
                MathUtil.clamp(pivotError * PIVOT_POWER_MULT, -1, 1), PIVOT_TOLERANCE);
        double extensionOutput = MathUtils.applyDeadzone(
                MathUtil.clamp(extensionError * EXTENSION_POWER_MULT, -1, 1), EXTENSION_TOLERANCE);
        // Move the arm
        arm.move(pivotOutput, extensionOutput);
    }

    @Override
    public boolean isFinished() {
        // This value is checked after execute() is ran. If true, end(false) is run.
        return Math.abs(pivotError) < PIVOT_TOLERANCE && Math.abs(extensionError) < EXTENSION_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        // This method runs once at the end. The interrupted parameter is true when the
        // command is stopped forcefully, IE not through isFinished().
        arm.stop();
    }

    /**
     * Calculates the error between the target value and the current value, and
     * writes them to the pivotError and extensionError variables.
     */
    private void calculateError() {
        pivotError = targetPosition.pivotAngle() - arm.getPosition().pivotAngle();
        extensionError = targetPosition.extensionLength() - arm.getPosition().extensionLength();
    }
}