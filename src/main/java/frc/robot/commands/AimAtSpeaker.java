package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Vision.LimeLight;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.helpers.MathUtils;
import frc.robot.subsystems.ShooterSubsystem;

public class AimAtSpeaker extends Command {
    ShooterSubsystem shooter;
    DoubleConsumer rotationalOverrideConsumer;
    Runnable rotationalOverrideDisabler;
    Supplier<Pose2d> botPoseSupplier;

    PIDController rotationalOverridePID = new PIDController(1.0 / 20, 0, 1 / 40);
    PIDController pivotPID = new PIDController(0.9, 0, 0);

    double rotError;

    Timer shotTimer;

    public AimAtSpeaker(ShooterSubsystem shooter, DoubleConsumer rotationalOverrideConsumer,
            Runnable rotationalOverrideDisabler, Supplier<Pose2d> botPoseSupplier) {
        this.shooter = shooter;
        this.rotationalOverrideConsumer = rotationalOverrideConsumer;
        this.rotationalOverrideDisabler = rotationalOverrideDisabler;
        this.botPoseSupplier = botPoseSupplier;

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
        if (LimelightHelpers.getTV(LimeLight.APRILTAG_DETECTOR) && isLookingAtRightTag()) {
            rotError = -LimelightHelpers.getTX(LimeLight.APRILTAG_DETECTOR);
        } else {
            rotError = 0;
        }

        rotationalOverrideConsumer.accept(rotationalOverridePID.calculate(rotError, 0));
        shooter.setPivot(getTargetAngle());

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
                && Math.abs(getTargetAngle().minus(shooter.getShooterAngle()).getDegrees()) < 1
                // And the shooter is spinning fast enough
                && shooter.getShooterVelocity() >= shooter.getShooterTargetVelocity() * 0.9;
    }

    private Translation2d speakerPos2d = MathUtils.adjustTranslation(new Translation2d(0, 5.55));
    private double speakerHeight = 2.0;

    private Rotation2d getTargetAngle() {
        Translation2d botTranslation = botPoseSupplier.get().getTranslation();
        double distanceFromSpeaker = botTranslation.getDistance(speakerPos2d);
        return new Rotation2d(distanceFromSpeaker, speakerHeight);
    }

    private boolean isLookingAtRightTag() {
        boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue;
        if (isRed) {
            return LimelightHelpers.getFiducialID(LimeLight.APRILTAG_DETECTOR) == 4;
        } else {
            return LimelightHelpers.getFiducialID(LimeLight.APRILTAG_DETECTOR) == 8;
        }
    }
}
