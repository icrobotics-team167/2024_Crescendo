package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class TestShooter extends Command {
    ShooterSubsystem shooter;
    public TestShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        setName("Test Shooter");
    }

    @Override
    public void execute() {
        shooter.runShooter();
    }
}
