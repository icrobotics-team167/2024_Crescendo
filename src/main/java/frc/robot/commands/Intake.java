package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Intake extends Command {
    ShooterSubsystem shooter;
    public Intake(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        setName("Intake");
    }

    @Override
    public void execute() {
        // TODO: Configure
        //shooter.setPivot(new Rotation2d());
        shooter.runIntake();
    }

    @Override
    public boolean isFinished() {
        return shooter.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopIntake();
    }
}
