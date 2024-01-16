package frc.robot.commands.teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AimShooter extends Command {
    ShooterSubsystem shooter;
    Supplier<Pose2d> botPoseSupplier;
    public AimShooter(ShooterSubsystem shooter, Supplier<Pose2d> botPoseSupplier) {
        this.shooter = shooter;
        this.botPoseSupplier = botPoseSupplier;
        addRequirements(shooter);
        setName("Aim");
    }
    
    @Override
    public void execute() {
        shooter.setPivot(calculateShotAngle());
        if (isInShotRange()) {
            shooter.shoot();
        }
    }

    public static Translation3d targetPos = new Translation3d();

    private Rotation2d calculateShotAngle() {
        // TODO: Implement
        throw new UnsupportedOperationException("Unimplemented method calculateShotAngle");
    }

    private boolean isInShotRange() {
        // TODO: Implement
        throw new UnsupportedOperationException("Unimplemented method isInShotRange");
    }
}
