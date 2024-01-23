package frc.robot.commands;

import java.util.function.DoubleConsumer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Vision.LimeLight;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;

public class AimAtSpeaker extends Command {
    ShooterSubsystem shooter;
    DoubleConsumer rotationalOverrideConsumer;
    Runnable rotationalOverrideDisabler;

    PIDController rotationalOverridePID = new PIDController(1.0 / 20, 0, 1 / 40);
    PIDController pivotPID = new PIDController(0.9, 0, 0);

    double rotError;
    double pivotError;

    Timer shotTimer;

    public AimAtSpeaker(ShooterSubsystem shooter, DoubleConsumer rotationalOverrideConsumer,
            Runnable rotationalOverrideDisabler) {
        this.shooter = shooter;
        this.rotationalOverrideConsumer = rotationalOverrideConsumer;
        this.rotationalOverrideDisabler = rotationalOverrideDisabler;

        addRequirements(shooter);
        setName("Aim At Speaker");
    }

    @Override
    public void initialize() {
        shotTimer.reset();
    }

    @Override
    public void execute() {
        shooter.runShooter();
        if (LimelightHelpers.getTV(LimeLight.APRILTAG_DETECTOR)
                && (LimelightHelpers.getFiducialID(LimeLight.APRILTAG_DETECTOR) == 8
                        || LimelightHelpers.getFiducialID(LimeLight.APRILTAG_DETECTOR) == 4)) {
            rotError = -LimelightHelpers.getTX(LimeLight.APRILTAG_DETECTOR);
            pivotError = LimelightHelpers.getTY(LimeLight.APRILTAG_DETECTOR);
        } else {
            rotError = 0;
            pivotError = 0;
        }

        rotationalOverrideConsumer.accept(rotationalOverridePID.calculate(rotError, 0));
        shooter.runPivot(pivotPID.calculate(pivotError, 0));

        if (isOkToShoot()) {
            shooter.runFeedOut();
            shotTimer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return shotTimer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupted) {
        rotationalOverrideDisabler.run();
        shooter.stopShooter();
        shooter.stopFeed();
        shotTimer.stop();
    }

    private boolean isOkToShoot() {
        // return false;
        // If rotational error is within tolerance
        return Math.abs(rotError) < 1
                // And pivot error is within tolerance
                && Math.abs(pivotError) < 1
                // And the shooter is spinning fast enough
                && shooter.getShooterVelocity() >= shooter.getShooterTargetVelocity() * 0.8;
    }
}
