package frc.robot.helpers;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Field;

/**
 * Because WPILib's built-in MathUtil class wasn't good enough.
 */
public class MathUtils {
    /**
     * Acceleration due to gravity, in m/s^2
     */
    public static final double GRAVITY = 9.80665;
    /**
     * Tada's magic number.
     */
    public static final double TADAS_MAGIC_NUMER = 0.4;

    /**
     * Gets the sign of a number.
     * 
     * @param num The number.
     * @return The sign. 0 if the number is 0, 1 is the number is positive, -1 if
     *         the number is negative.
     */
    public static int getSign(double num) {
        if (num == 0) {
            return 0;
        }
        return num > 0 ? 1 : -1;
    }

    /**
     * If the robot is on the Red alliance, flip the robot pose to adjust for that.
     * 
     * @param pose The current robot pose.
     * @return If the robot is on the Red alliance, returns the transformed pose. If
     *         the robot is on the Blue alliance, does nothing to pose.
     */
    public static Pose2d adjustPose(Pose2d pose) {
        return new Pose2d(adjustTranslation(pose.getTranslation()), adjustRotation(pose.getRotation()));
    }

    /**
     * If the robot is on the Red alliance, flip the robot translation to adjust for
     * that.
     * 
     * @param translation The current robot translation.
     * @return If the robot is on the Red alliance, returns the transformed
     *         translation. If the robot is on the Blue alliance, does nothing to
     *         translation.
     */
    public static Translation2d adjustTranslation(Translation2d translation) {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return translation;
        }
        double poseX = Field.FIELD_LENGTH - translation.getX();
        double poseY = Field.ASSYMETRICAL_FIELD ? Field.FIELD_WIDTH - translation.getY() : translation.getY();
        return new Translation2d(poseX, poseY);
    }

    /**
     * If the robot is on the Red alliance, flip the robot rotation to adjust for
     * that.
     * 
     * @param rotation The current robot rotation.
     * @return If the robot is on the Red alliance, returns the transformed
     *         rotation. If the robot is on the Blue alliance, does nothing to
     *         rotation.
     */
    public static Rotation2d adjustRotation(Rotation2d rotation) {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return rotation;
        }
        return new Rotation2d(Math.PI - rotation.getRadians());
    }

    /**
     * If the robot is on the Red alliance, flip PathPlanner trajectories to adjust
     * for that.
     * 
     * @param trajectory The trajectory to be followed.
     * @return If the robot is on the Red alliance, returns the transformed
     *         trajectory. if the robot is on the Blue alliance, does nothing to
     *         trajectory.
     */
    public static PathPlannerTrajectory adjustTrajectory(PathPlannerTrajectory trajectory) {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return trajectory;
        }
        List<State> states = trajectory.getStates();
        for (State state : states) {
            state.positionMeters = adjustTranslation(state.positionMeters);
            state.heading = adjustRotation(state.heading);
            state.targetHolonomicRotation = adjustRotation(state.targetHolonomicRotation);
        }
        return trajectory;
    }

    /**
     * Applies a deadzone to a value, putting a sharp drop where the cutoff is. This
     * is unlike WPILib's MathUtil.applyDeadband() where values not set to 0 are
     * scaled to be continuous.
     * 
     * @param value        The value to apply the deadzone to.
     * @param deadzoneSize If value is less than this, returns 0.
     * @return The value with the deadzone applied.
     */
    public static double applyDeadzone(double value, double deadzoneSize) {
        if (Math.abs(value) < deadzoneSize) {
            return 0;
        }
        return value;
    }

    /**
     * If the robot is on the red alliance, flips the pose.
     * 
     * @param rawPose The Pose2d of the robot. Origin must be on WPI Blue.
     * @return If the robot is on red, flipped pose, maintaining a WPI Blue Origin.
     *         If it's on blue, no changes made to rawPose.
     */
    public static Pose2d toAlliancePose(Pose2d rawPose) {
        if (RobotContainer.isRedAlliance()) {
            return new Pose2d(Constants.Field.FIELD_LENGTH - rawPose.getX(), rawPose.getY(),
                    Rotation2d.fromDegrees(180 - rawPose.getRotation().getDegrees()));
        }
        return rawPose;
    }

    /**
     * If the robot is on the red alliance, flips the translation.
     * 
     * @param rawTranslation The Translation2d of the robot. Origin must be on WPI
     *                       Blue.
     * @return If the robot is on red, flipped translation, maintaining a WPI Blue
     *         origin. If it's on blue, no changes made to rawTranslation.
     */
    public static Translation2d toAllianceTranslation(Translation2d rawTranslation) {
        if (RobotContainer.isRedAlliance()) {
            return new Translation2d(Constants.Field.FIELD_LENGTH - rawTranslation.getX(), rawTranslation.getY());
        }
        return rawTranslation;
    }
}
