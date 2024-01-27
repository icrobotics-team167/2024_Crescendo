package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.MathUtils;
import frc.robot.subsystems.ShooterSubsystem;

public class AimAtSpeaker extends Command {
    private ShooterSubsystem shooter;
    private Consumer<Rotation2d> rotationalOverrideConsumer;
    private Supplier<Pose2d> botPoseSupplier;

    private double rotError;

    private Timer shotTimer;

    public AimAtSpeaker(ShooterSubsystem shooter, Consumer<Rotation2d> rotationalOverrideConsumer,
            Supplier<Pose2d> botPoseSupplier) {
        this.shooter = shooter;
        this.rotationalOverrideConsumer = rotationalOverrideConsumer;
        this.botPoseSupplier = botPoseSupplier;
        shotTimer = new Timer();

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
        rotationalOverrideConsumer.accept(getTargetYaw());
        shooter.setPivot(getTargetPivot());
        if (isOkToShoot()) {
            // shooter.runFeedOut();
            // shotTimer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return shotTimer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupted) {
        rotationalOverrideConsumer.accept(null);
        shooter.stopShooter();
        shooter.stopFeed();
        shotTimer.stop();
    }

    private boolean isOkToShoot() {
        // return false;
        // If rotational error is within tolerance
        return Math.abs(rotError) < 1
                // And pivot error is within tolerance
                && Math.abs(getTargetPivot().minus(shooter.getShooterAngle()).getDegrees()) < 1
                // And the shooter is spinning fast enough
                && shooter.getShooterVelocity() >= shooter.getShooterTargetVelocity() * 0.9;
    }

    private Translation2d speakerPos2d = MathUtils.adjustTranslation(new Translation2d(0, 5.55));
    private double speakerHeight = 2.0;
    StructPublisher<Pose3d> speakerPosPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("AimAtSpeaker/SpeakerPos", Pose3d.struct).publish();

    private Rotation2d getTargetPivot() {
        Translation2d botTranslation = botPoseSupplier.get().getTranslation();
        double distanceFromSpeaker = botTranslation.getDistance(speakerPos2d);
        return new Rotation2d(distanceFromSpeaker, speakerHeight);
    }

    private Rotation2d getTargetYaw() {
        speakerPosPublisher
                .accept(new Pose3d(speakerPos2d.getX(), speakerPos2d.getY(), speakerHeight, new Rotation3d()));
        Translation2d botTranslation = botPoseSupplier.get().getTranslation();
        Translation2d botRelativeToTarget = speakerPos2d.minus(botTranslation);
        return botRelativeToTarget.getAngle();
    }
}
