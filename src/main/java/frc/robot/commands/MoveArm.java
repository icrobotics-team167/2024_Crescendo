package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/**
 * A teleop command to move the arm.
 */
public class MoveArm extends Command {
    ArmSubsystem arm;
    DoubleSupplier extendControl, pivotControl;

    /**
     * Constructs a new MoveArm command.
     * 
     * @param arm           The arm subsytem.
     * @param extendControl The double supplier providing a control input for
     *                      extending. 1.0 is full speed out, -1.0 is full speed in.
     * @param pivotControl  The double supplier providing a control input for
     *                      pivoting. 1.0 is full speed down, -1.0 is full speed up.
     */
    public MoveArm(ArmSubsystem arm, DoubleSupplier extendControl, DoubleSupplier pivotControl) {
        this.arm = arm;
        this.extendControl = extendControl;
        this.pivotControl = pivotControl;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.move(pivotControl.getAsDouble(), extendControl.getAsDouble());
    }
}
