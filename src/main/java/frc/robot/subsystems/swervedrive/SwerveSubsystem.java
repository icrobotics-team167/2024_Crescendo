package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrivebase;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrivebase swerveDrive;

    public SwerveSubsystem() {
        swerveDrive = new SwerveDrivebase();
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::robotRelativeDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                Constants.Robot.Auto.PATH_FOLLOWER_CONFIG,
                this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Drives the robot, relative to itself.
     * 
     * @param velocityCommand The commanded velocities of the robot, as a
     *                        ChassisSpeeds object. Is Forward-Left-CCW positive.
     */
    public void robotRelativeDrive(ChassisSpeeds velocityCommand) {
        drive(velocityCommand, false);
    }

    /**
     * Drives the robot, relative to the driver station.
     * 
     * @param velocityCommand The commanded velocities of the robot, as a
     *                        ChassisSpeeds object. Is Forward-Left-CCW positive.
     */
    public void fieldRelativeDrive(ChassisSpeeds velocityCommand) {
        drive(velocityCommand, true);
    }

    /**
     * Drives the robot.
     * 
     * @param velocityCommand The commanded velocities of the robot, as a
     *                        ChassisSpeeds object. Is Forward-Left-CCW positive.
     * @param fieldRelative   Whether or not the frame of reference is to the robot,
     *                        or to the driver station.
     */
    public void drive(ChassisSpeeds velocityCommand, boolean fieldRelative) {
        swerveDrive.drive(velocityCommand, fieldRelative);
    }

    public void toggleSlowMode() {
        swerveDrive.toggleSlowMode();
    }

    public void resetYaw() {
        swerveDrive.resetYaw();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetPose(Pose2d pose) {
        swerveDrive.resetPose(pose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    public double getMaxVel() {
        return swerveDrive.getAbsoluteMaxVel();
    }
}
