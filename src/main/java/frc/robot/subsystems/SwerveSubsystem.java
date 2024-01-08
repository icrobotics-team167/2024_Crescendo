package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.swerve.SwerveDrivebase;

/**
 * A SubsystemBase class to implement the swerve drive.
 */
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrivebase swerveDrive;

    /**
     * Constructs a new SwerveSubsystem.
     */
    public SwerveSubsystem() {
        swerveDrive = new SwerveDrivebase();
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::robotRelativeDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig(
                        Constants.Robot.Auto.translationalPIDs, // Translation PID constants
                        Constants.Robot.Auto.rotationalPIDs, // Rotation PID constants
                        getAbsoluteMaxVel(), // Max module speed of the slowest module, in m/s.
                        getDrivebaseRadius(), // Drive base radius in meters.
                                              // Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                RobotContainer::isRedAlliance,
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
    private void drive(ChassisSpeeds velocityCommand, boolean fieldRelative) {
        swerveDrive.drive(velocityCommand, fieldRelative);
    }

    /**
     * Puts the drivebase into slow mode.
     */
    public void setSlowMode() {
        swerveDrive.setSlowMode(true);
    }

    /**
     * Puts the drivebase into normal mode.
     */
    public void unsetSlowMode() {
        swerveDrive.setSlowMode(false);
    }

    /**
     * Locks the wheels so that the robot won't move. Use to lock robot motion after
     * a match or when an enemy is pushing the robot.
     */
    public void lockMotion() {
        swerveDrive.lockMotion();
    }

    /**
     * Unlocks wheels.
     */
    public void unlockMotion() {
        swerveDrive.unlockMotion();
    }

    /**
     * Sets whether the drive wheels are in brake mode or not.
     * 
     * @param brake True for brake, false for coast.
     */
    public void setWheelBrake(boolean brake) {
        swerveDrive.setWheelBrake(brake);
    }

    /**
     * Sets the wheels to face forwards.
     */
    public void setWheelsForward() {
        swerveDrive.setWheelsForward();
    }

    public void setIndividualModule(int moduleID, SwerveModuleState desiredState) {
        swerveDrive.setIndividualModule(moduleID, desiredState);
    }

    /**
     * Resets the rotation of the robot to be forwards.
     */
    public void resetRotation() {
        swerveDrive.resetRotation();
    }

    /**
     * Gets the current pose of the robot. (0,0) is the back right corner of
     * the blue driver station/the bottom left corner of the PathPlanner app,
     * depending on your perspective. 0 degrees rotation is facing away from the
     * blue driver station, 180 degrees is facing away from the red driver station.
     * 
     * @return The robot pose.
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Resets the robot pose to a specified position.
     * 
     * @param pose The specified position.
     */
    public void resetPose(Pose2d pose) {
        swerveDrive.resetPose(pose);
    }

    /**
     * Gets the current robot velocity as a ChassisSpeeds object. Is robot relative.
     * 
     * @return The current robot velocity.
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Gets the absolute max velocity of the drivebase. Not neccesarily the max
     * configured speed.
     * 
     * @return The max velocity, in m/s
     */
    public double getAbsoluteMaxVel() {
        return swerveDrive.getAbsoluteMaxVel();
    }

    /**
     * Gets the distance of the furthest module from the center of the robot.
     * 
     * @return The radius, in meters.
     */
    public double getDrivebaseRadius() {
        return swerveDrive.getDrivebaseRadius();
    }

    /**
     * Sends telemetry data.
     */
    public void sendTelemetry() {
        swerveDrive.sendTelemetry();
    }
}
