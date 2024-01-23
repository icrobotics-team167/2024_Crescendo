package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AimAtSpeaker extends Command {
    ShooterSubsystem shooter;
    Supplier<Pose2d> botPoseSupplier;

    public AimAtSpeaker(ShooterSubsystem shooter, Supplier<Pose2d> botPoseSupplier) {
        this.shooter = shooter;
        this.botPoseSupplier = botPoseSupplier;
        addRequirements(shooter);
        setName("Aim At Speaker");
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
