package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm extends Command {
    ArmSubsystem arm;
    DoubleSupplier extendControl, pivotControl;
    public MoveArm( ArmSubsystem arm, DoubleSupplier extendControl, DoubleSupplier pivotControl) {
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
