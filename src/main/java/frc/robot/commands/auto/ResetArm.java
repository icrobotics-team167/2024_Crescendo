package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ResetArm extends Command {
    private ArmSubsystem arm;

    public ResetArm(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        arm.move(1, -1);
    }

    @Override
    public boolean isFinished() {
        return arm.isTooFarUp() && arm.isTooFarIn();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
