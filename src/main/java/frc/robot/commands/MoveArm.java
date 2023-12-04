package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArm extends Command {
    ArmSubsystem arm;
    DoubleSupplier extend, pivot;
    public MoveArm( ArmSubsystem arm, DoubleSupplier extend, DoubleSupplier pivot) {
        this.arm = arm;
        this.extend = extend;
        this.pivot = pivot;
    }
    @Override
    public void execute() {
        arm.move(pivot.getAsDouble(), extend.getAsDouble());
    }
}
