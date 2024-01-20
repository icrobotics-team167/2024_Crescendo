package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.helpers.MathUtils;
import frc.robot.helpers.Telemetry;
import frc.robot.helpers.Telemetry.Verbosity;
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
    public void execute() {
        if (!shooter.hasRing()) {
            return;
        }
        shooter.runShooter();
        shooter.setPivot(calculateShotAngle());
        if (Telemetry.sendBoolean("AimAtSpeaker/isOkToShoot", isOkToShoot(), Verbosity.HIGH)) {
            shooter.runIntake();
        }
    }

    /**
     * The position of the goal, on the blue alliance.
     * TODO: Tune
     */
    public static Translation3d targetPos = new Translation3d(0, 5.5, 2.04);
    /**
     * The width of the target.
     */
    public static double targetWidth = 1.05;

    /**
     * Calculate the angle we would need to launch at to send the ring into the
     * goal.
     * 
     * @return The angle we need to launch at.
     */
    private Rotation2d calculateShotAngle() {
        Translation2d target = MathUtils.toAllianceTranslation(targetPos.toTranslation2d());
        double distance = calculateDistanceFromTarget(botPoseSupplier.get(), target);
        double height = targetPos.getZ();
        return new Rotation2d(Math.atan(height / distance)); // TODO: Is this math right?
    }

    private double calculateDistanceFromTarget(Pose2d botPose, Translation2d targetPose) {
        return botPose.getTranslation().getDistance(targetPose);
    }

    /**
     * Runs a bunch of checks to see if we can shoot the ring and have it land in
     * the goal.
     * 
     * @return If the ring will land in the goal if we shoot right now.
     */
    private boolean isOkToShoot() {
        // Can we make the shot without hitting the slope?
        boolean angleNotTooLow = Telemetry.sendNumber("AimAtSpeaker/shotAngle", calculateShotAngle().getDegrees(),
                Verbosity.HIGH) > 14;

        // Is the bot pointed at the target?
        Rotation2d[] shotRotationRange = calculateShotRotationRange();
        Telemetry.sendNumber("AimAtSpeaker/shotRotationLowerBound", shotRotationRange[0].getDegrees(), Verbosity.HIGH);
        Telemetry.sendNumber("AimAtSpeaker/shotRotationUpperBound", shotRotationRange[1].getDegrees(), Verbosity.HIGH);
        Rotation2d botRotation = botPoseSupplier.get().getRotation();
        boolean pointedAtTarget = botRotation.getDegrees() > shotRotationRange[0].getDegrees()
                && botRotation.getDegrees() < shotRotationRange[1].getDegrees();

        // Is the shooter spinning fast enough?
        boolean shooterSpinningFastEnough = Telemetry.sendNumber("AimAtSpeaker/shooterSpeedPercent",
                shooter.getShooterVelocity(), Verbosity.HIGH) > 4;

        // Is the shooter at roughly the right angle?
        boolean shooterAtRightAngle = Math.abs(shooter.shooterAngle().minus(calculateShotAngle()).getDegrees()) < 2;

        return angleNotTooLow && pointedAtTarget && shooterSpinningFastEnough && shooterAtRightAngle;
    }

    /**
     * Calculates the lower and upper bound of angles that the bot can face when
     * shooting.
     * 
     * @return An array of Rotation2ds. Element 0 is the lower bound, 1 is the upper
     *         bound.
     */
    private Rotation2d[] calculateShotRotationRange() {
        double shotYLowerBound = targetPos.getY() - (targetWidth / 2);
        double shotYUpperBound = targetPos.getY() + (targetWidth / 2);
        Translation2d botTranslation = botPoseSupplier.get().getTranslation();
        // TODO: Is the math right?
        Translation2d target = MathUtils.toAllianceTranslation(targetPos.toTranslation2d());
        Rotation2d shotRotationLowerBound = new Translation2d(target.getX(), shotYLowerBound).minus(botTranslation)
                .getAngle();
        Rotation2d shotRotationUpperBound = new Translation2d(target.getX(), shotYUpperBound).minus(botTranslation)
                .getAngle();
        if (shotRotationLowerBound.getDegrees() < shotRotationLowerBound.getDegrees()) {
            return new Rotation2d[] {
                    shotRotationLowerBound,
                    shotRotationUpperBound
            };
        } else {
            return new Rotation2d[] {
                    shotRotationUpperBound,
                    shotRotationLowerBound
            };
        }
    }
}
