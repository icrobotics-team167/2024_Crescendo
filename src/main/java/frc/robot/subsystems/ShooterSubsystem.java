package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstraction.motors.RevNEO500;
import frc.robot.helpers.Telemetry;
import frc.robot.helpers.Telemetry.Verbosity;
import frc.robot.subsystems.shooter.Intake;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterSubsystem extends SubsystemBase {
    Pivot pivot;
    Intake intake;
    Shooter shooter;

    public ShooterSubsystem() {
        // TODO: Configure
        pivot = new Pivot(new RevNEO500(11), new RevNEO500(10), null);
        // intake = new Intake(null);
        // shooter = new Shooter(new RevNEO500(10), new RevNEO500(11), 0.0);
    }

    @Override
    public void periodic() {
        Telemetry.sendNumber("Pivot/actualAngle", getPivotAngle().getDegrees(), Verbosity.HIGH);
    }

    public void setPivot(Rotation2d angle) {
        pivot.setDesiredAngle(angle);
    }

    public void runPivot(double setPoint) {
        pivot.move(setPoint);
    }

    public Rotation2d getPivotAngle() {
        return pivot.getAngle();
    }

    public void runIntake() {
        // intake.run();
    }

    public void stopIntake() {
        // intake.stop();
    }

    public void runFeedIn() {
        // intake.runFeedIn();
    }

    public void runFeedOut() {
        // intake.runFeedOut();
    }

    public void stopFeed() {
        // intake.stopFeed();
    }

    public void runShooter() {
        // shooter.run();
    }

    public void runShooterRaw(Measure<Voltage> volts) {
        // shooter.runVolts(volts.magnitude());
    }

    public void stopShooter() {
        // shooter.stop();
    }

    public boolean hasRing() {
        // TODO: Implement
        return false;
    }

    public double getShooterVelocity() {
        // return shooter.getVelocity();
        return 0;
    }

    public double getShooterTargetVelocity() {
        // return shooter.getTargetVelocity();
        return 0;
    }

    public double getShooterPosition() {
        // return shooter.getPosition();
        return 0;
    }

    public Rotation2d getShooterAngle() {
        return pivot.getAngle();
        // return new Rotation2d();
    }
}
