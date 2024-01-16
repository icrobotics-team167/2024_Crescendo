package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    Pivot pivot;
    Intake intake;
    Shooter shooter;
    public ShooterSubsystem() {
        // TODO: Configure
        pivot = new Pivot(null, null, null);
        intake = new Intake(null);
        shooter = new Shooter(null, null);
    }

    public void setPivot(Rotation2d angle) {
        pivot.setDesiredAngle(angle);
    }

    public void runIntake() {
        intake.run();
    }

    public void stopIntake() {
        intake.stop();
    }

    public void runShooter() {
        shooter.run();
    }

    public void stopShooter() {
        shooter.stop();
    }

    public boolean hasRing() {
        // TODO: Implement
        return false;
    }

    public double shooterVelocityPercentage() {
        return shooter.getVelocityPercentage();
    }
}
