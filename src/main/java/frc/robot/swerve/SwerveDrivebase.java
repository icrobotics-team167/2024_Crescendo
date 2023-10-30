package frc.robot.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.abstraction.encoders.AnalogAbsoluteEncoder;
import frc.robot.abstraction.imus.AbstractIMU;
import frc.robot.abstraction.imus.Pigeon2IMU;
import frc.robot.abstraction.motors.RevNEO500;
import frc.robot.Constants.Robot.SwerveDrive;
import frc.robot.Constants.Robot.SwerveDrive.Modules;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveDrivebase {
    // Driving math
    /**
     * The module kinematics.
     */
    private final SwerveDriveKinematics kinematics;
    /**
     * The modules.
     */
    private final Module modules[];
    /**
     * The module positions.
     */
    private final SwerveModulePosition[] modulePositions;
    /**
     * Whether or not slow mode is enabled.
     */
    private boolean slowMode = false;

    // Odometry
    /**
     * The intertial measurement unit of the robot.
     */
    private final AbstractIMU imu;
    /**
     * The pose estimator for the robot. Combines regular wheel odometry with vision
     * proccesing based pose estimation.
     */
    private final SwerveDrivePoseEstimator poseEstimator;
    /**
     * Trustworthiness of the internal model of how motors should be moving Measured
     * in expected standard deviation
     * (meters of position and degrees of rotation)
     */
    public Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    /**
     * Trustworthiness of the vision system Measured in expected standard deviation
     * (meters of position and degrees of
     * rotation)
     */
    public Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    /**
     * The Notifier thread to keep odometry up to date.
     */
    private final Notifier odometryThread;
    /**
     * Thread lock for odometryThread to ensure thread safety.
     */
    private final Lock odometryLock = new ReentrantLock();

    /**
     * A representation of the field
     */
    private Field2d field = new Field2d();

    public SwerveDrivebase() {
        imu = new Pigeon2IMU(10);
        this.imu.factoryDefault();
        this.imu.clearStickyFaults();

        kinematics = new SwerveDriveKinematics(
                Modules.Positions.FRONT_LEFT_POS,
                Modules.Positions.FRONT_RIGHT_POS,
                Modules.Positions.BACK_LEFT_POS,
                Modules.Positions.BACK_RIGHT_POS);
        modules = new Module[] {
                new Module(0,
                        new RevNEO500(Modules.IDs.FRONT_LEFT_DRIVE),
                        new RevNEO500(Modules.IDs.FRONT_LEFT_TURN),
                        new AnalogAbsoluteEncoder(Modules.IDs.FRONT_LEFT_ENCODER),
                        Modules.EncoderOffsets.FRONT_LEFT_OFFSET),
                new Module(1,
                        new RevNEO500(Modules.IDs.FRONT_RIGHT_DRIVE),
                        new RevNEO500(Modules.IDs.FRONT_RIGHT_TURN),
                        new AnalogAbsoluteEncoder(Modules.IDs.FRONT_RIGHT_ENCODER),
                        Modules.EncoderOffsets.FRONT_RIGHT_OFFSET),
                new Module(2,
                        new RevNEO500(Modules.IDs.BACK_LEFT_DRIVE),
                        new RevNEO500(Modules.IDs.BACK_LEFT_TURN),
                        new AnalogAbsoluteEncoder(Modules.IDs.BACK_LEFT_ENCODER),
                        Modules.EncoderOffsets.BACK_LEFT_OFFSET),
                new Module(3,
                        new RevNEO500(Modules.IDs.BACK_RIGHT_DRIVE),
                        new RevNEO500(Modules.IDs.BACK_RIGHT_TURN),
                        new AnalogAbsoluteEncoder(Modules.IDs.BACK_RIGHT_ENCODER),
                        Modules.EncoderOffsets.BACK_RIGHT_OFFSET),
        };
        modulePositions = new SwerveModulePosition[] {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition(),
        };

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getYaw(),
                modulePositions,
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);
        odometryThread = new Notifier(this::updateOdometry);
        odometryThread.startPeriodic(0.02);
    }

    /**
     * Drives the robot, relative to itself.
     * 
     * @param velocityCommand The commanded linear velocity of the robot, in meters
     *                        per second. Positive x is forward, positive y is left.
     */
    public void robotRelativeDrive(ChassisSpeeds velocityCommand) {
        drive(velocityCommand, false);
    }

    /**
     * Drives the robot, relative to the field.
     * 
     * @param velocityCommand The commanded linear velocity of the robot, in meters
     *                        per second. Positive x is away from the driver
     *                        station, positive y is to the left relative to the
     *                        driver station.
     */
    public void fieldOrientedDrive(ChassisSpeeds velocityCommand) {
        drive(velocityCommand, true);
    }

    /**
     * Drives the robot.
     * 
     * @param velocityCommand The commanded linear velocity of the robot, in meters
     *                        per second. Positive x is forwards, (either from the
     *                        robot or from the driver station) positve y is left
     *                        (either from the robot or from the driver station)
     * @param fieldRelative   Driving mode. True for field relative, false for robot
     *                        relative.
     */
    public void drive(ChassisSpeeds velocityCommand, boolean fieldRelative) {
        if (fieldRelative) {
            velocityCommand = ChassisSpeeds.fromFieldRelativeSpeeds(velocityCommand, getYaw());
        }

        if (slowMode) {
            velocityCommand = new ChassisSpeeds(
                    velocityCommand.vxMetersPerSecond * SwerveDrive.SLOWMODE_MULT,
                    velocityCommand.vyMetersPerSecond * SwerveDrive.SLOWMODE_MULT,
                    velocityCommand.omegaRadiansPerSecond * SwerveDrive.SLOWMODE_MULT);
        }

        // When driving and turning at the same time, the robot slightly drifts. This
        // compensates for that.
        // TODO: See if this is actually neccesary.
        // Stolen code from 254
        // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5
        double dtConstant = 0.009;
        Pose2d robotPoseVel = new Pose2d(velocityCommand.vxMetersPerSecond * dtConstant,
                velocityCommand.vyMetersPerSecond * dtConstant,
                Rotation2d.fromRadians(velocityCommand.omegaRadiansPerSecond * dtConstant));
        Twist2d twistVel = poseLog(robotPoseVel);

        velocityCommand = new ChassisSpeeds(twistVel.dx / dtConstant, twistVel.dy / dtConstant,
                twistVel.dtheta / dtConstant);

        // Drive modules
        SwerveModuleState[] moduleDesiredStates = kinematics.toSwerveModuleStates(velocityCommand);
        // If the commanded module speeds is too fast, slow down
        SwerveDriveKinematics.desaturateWheelSpeeds(
                moduleDesiredStates,
                getRobotVelocity(),
                getAbsoluteMaxVel(),
                getMaxTranslationalVel(),
                getMaxRotVel());
        // SwerveDriveKinematics.desaturateWheelSpeeds(moduleDesiredStates,
        // getRobotVelocity(), dtConstant, rotation, rotation);
        for (Module module : modules) {
            module.setDesiredState(moduleDesiredStates[module.moduleNumber]);
        }
    }

    /**
     * Gets the current rotation of the robot.
     * 
     * @return The current rotation as a Rotation3d object.
     */
    public Rotation3d getRotation() {
        return imu.getRotation3d();
    }

    /**
     * Gets the current yaw of the robot.
     * 
     * @return The yaw as a Rotation2d object.
     */
    public Rotation2d getYaw() {
        return new Rotation2d(getRotation().getZ());
    }

    /**
     * Gets the current pitch of the robot.
     * 
     * @return The pitch as a Rotation2d object.
     */
    public Rotation2d getPitch() {
        return new Rotation2d(getRotation().getX());
    }

    /**
     * Gets the current roll of the robot.
     * 
     * @return The roll as a Rotation2d object.
     */
    public Rotation2d getRoll() {
        return new Rotation2d(getRotation().getY());
    }

    public ChassisSpeeds getRobotVelocity() {
        return kinematics.toChassisSpeeds(getStates());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (Module module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (Module module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    public void updateOdometry() {
        odometryLock.lock();
        try {
            poseEstimator.update(getYaw(), modulePositions);
            field.setRobotPose(poseEstimator.getEstimatedPosition());
        } catch (Exception e) {
            odometryLock.unlock();
            throw e;
        }
        odometryLock.unlock();
    }

    public void addVisionMeasurement(Pose2d robotPose, double timestamp) {
        odometryLock.lock();
        poseEstimator.addVisionMeasurement(robotPose, timestamp);
        odometryLock.unlock();
        imu.setOffset(
                new Rotation3d(
                        getRoll().getRadians(),
                        getPitch().getRadians(),
                        robotPose.getRotation().getRadians()));
    }

    public void setOdometry(Pose2d pose) {
        odometryLock.lock();
        poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
        odometryLock.unlock();
        kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, pose.getRotation()));
    }

    public void toggleSlowMode() {
        if (slowMode) {
            slowMode = false;
        } else {
            slowMode = true;
        }
    }

    public void resetYaw() {
        imu.setOffset(
            new Rotation3d()
        );
    }

    /**
     * Gets the absolute max velocity of a module.
     * 
     * @return The max velocity, in m/s
     */
    public double getAbsoluteMaxVel() {
        return modules[0].getMaxVel();
    }

    /**
     * Gets the max translational velocity of the drivebase.
     * 
     * @return The max velocity, in m/s
     */
    public double getMaxTranslationalVel() {
        return SwerveDrive.MAX_TRANSLATIONAL_VEL;
    }

    /**
     * Gets the max rotational velocity of the drivebase.
     * 
     * @return The max velocity, in radians/s.
     */
    public double getMaxRotVel() {
        return SwerveDrive.MAX_ROTATIONAL_VEL;
    }

    /**
     * Logical inverse of the Pose exponential from 254. Taken from team 3181.
     *
     * @param transform Pose to perform the log on.
     * @return {@link Twist2d} of the transformed pose.
     */
    private Twist2d poseLog(final Pose2d transform) {

        final double kEps = 1E-9;
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().getCos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
        }
        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta,
                        -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
    }
}
