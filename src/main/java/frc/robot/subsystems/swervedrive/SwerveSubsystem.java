package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveDrivebase;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrivebase swerveDrive;

    public SwerveSubsystem() {
        swerveDrive = new SwerveDrivebase();
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
