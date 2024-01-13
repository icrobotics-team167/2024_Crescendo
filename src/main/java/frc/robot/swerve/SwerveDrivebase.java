package frc.robot.swerve;

// Like almost a full screen's worth of import lines
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.abstraction.encoders.AnalogAbsoluteEncoder;
import frc.robot.abstraction.imus.AbstractIMU;
import frc.robot.abstraction.imus.Pigeon2IMU;
import frc.robot.abstraction.motors.RevNEO500;
import frc.robot.Constants.Driving;
import frc.robot.Constants.Robot.SwerveDrive;
import frc.robot.Constants.Robot.SwerveDrive.Modules;
import frc.robot.Constants.Vision;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.helpers.MathUtils;
import frc.robot.helpers.Telemetry;
import frc.robot.helpers.Telemetry.Verbosity;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * A class for running ths swerve drivebase.
 */
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
     * Whether or not slow mode is enabled.
     */
    private boolean slowMode = false;
    /**
     * Whether or not motion is locked.
     */
    private boolean motionLocked = false;

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
     * Trustworthiness of the internal model of how motors should be moving.
     * Measured
     * in expected standard deviation
     * (meters of position and degrees of rotation)
     */
    public Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    /**
     * Trustworthiness of the vision system. Measured in expected standard deviation
     * (meters of position and degrees of
     * rotation)
     */
    public Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.3, 0.3, 0.3);
    /**
     * The Notifier thread to keep odometry up to date.
     */
    private final Notifier odometryThread;
    /**
     * The Notifier thread to keep vision based pose estimation up to date.
     */
    private final Notifier visionThread;
    /**
     * Thread lock to ensure thread safety.
     */
    private final Lock odometryLock = new ReentrantLock();

    /**
     * Constructs a new swerve drivebase. THIS IS NOT A SUBSYSTEM OBJECT.
     */
    public SwerveDrivebase() {
        // Initialize the intertial measurement unit.
        // Uses an abstraction layer so it's easy to swap out IMUs.
        imu = new Pigeon2IMU(10);
        this.imu.factoryDefault();
        this.imu.clearStickyFaults();

        // Initialize kinematics.
        kinematics = new SwerveDriveKinematics(
                Modules.Positions.FRONT_LEFT_POS,
                Modules.Positions.FRONT_RIGHT_POS,
                Modules.Positions.BACK_LEFT_POS,
                Modules.Positions.BACK_RIGHT_POS);

        // Initialize modules.
        // Motors and encoders use an abstraction layer so it's easy to swap them out.
        modules = new Module[] {
                new Module(0,
                        new RevNEO500(Modules.IDs.FRONT_LEFT_DRIVE),
                        new RevNEO500(Modules.IDs.FRONT_LEFT_TURN),
                        new AnalogAbsoluteEncoder(Modules.IDs.FRONT_LEFT_ENCODER,
                                Rotation2d.fromDegrees(Modules.EncoderOffsets.FRONT_LEFT_OFFSET))),
                new Module(1,
                        new RevNEO500(Modules.IDs.FRONT_RIGHT_DRIVE),
                        new RevNEO500(Modules.IDs.FRONT_RIGHT_TURN),
                        new AnalogAbsoluteEncoder(Modules.IDs.FRONT_RIGHT_ENCODER,
                                Rotation2d.fromDegrees(Modules.EncoderOffsets.FRONT_RIGHT_OFFSET))),
                new Module(2,
                        new RevNEO500(Modules.IDs.BACK_LEFT_DRIVE),
                        new RevNEO500(Modules.IDs.BACK_LEFT_TURN),
                        new AnalogAbsoluteEncoder(Modules.IDs.BACK_LEFT_ENCODER,
                                Rotation2d.fromDegrees(Modules.EncoderOffsets.BACK_LEFT_OFFSET))),
                new Module(3,
                        new RevNEO500(Modules.IDs.BACK_RIGHT_DRIVE),
                        new RevNEO500(Modules.IDs.BACK_RIGHT_TURN),
                        new AnalogAbsoluteEncoder(Modules.IDs.BACK_RIGHT_ENCODER,
                                Rotation2d.fromDegrees(Modules.EncoderOffsets.BACK_RIGHT_OFFSET))),
        };
        resetStates();

        // Initialize odometry.
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getYaw(),
                getModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);

        // Spin up threads to update odometry.
        odometryThread = new Notifier(this::updateOdometry);
        odometryThread.startPeriodic(0.02);
        visionThread = new Notifier(this::addLLVisionMeasurement);
        visionThread.startPeriodic(0.04);
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
     *                        robot or from the driver station) positve y is left.
     *                        (either from the robot or from the driver station)
     *                        Rotation is CCW+.
     * @param fieldRelative   Driving mode. True for field relative, false for robot
     *                        relative.
     */
    public void drive(ChassisSpeeds velocityCommand, boolean fieldRelative) {
        // If the wheels are locked, don't move at all.
        if (motionLocked) {
            return;
        }
        // For some reason if we don't do this it's CW+ instead of CCW+
        // TODO: Fix root cause to not have to do this
        velocityCommand.omegaRadiansPerSecond *= -1;
        // If the velocity command is field relative, convert it to robot relative
        // speeds.
        if (fieldRelative) {
            velocityCommand = ChassisSpeeds.fromFieldRelativeSpeeds(velocityCommand, getYaw());
        }

        // If we're in slow mode, apply a velocity multiplier.
        if (slowMode) {
            velocityCommand = new ChassisSpeeds(
                    velocityCommand.vxMetersPerSecond * Driving.SLOWMODE_MULT,
                    velocityCommand.vyMetersPerSecond * Driving.SLOWMODE_MULT,
                    velocityCommand.omegaRadiansPerSecond * Driving.SLOWMODE_MULT);
        }

        // Due to how converting continous velocity inputs into discrete module speeds
        // works, sometimes the robot will drift when moving and turning at the same
        // time. This compensates for that.
        velocityCommand = ChassisSpeeds.discretize(velocityCommand, 0.02);
        // Calculate module states.
        SwerveModuleState[] moduleDesiredStates = kinematics.toSwerveModuleStates(velocityCommand);
        // If the commanded module speeds is too fast, slow down
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleDesiredStates, velocityCommand, getAbsoluteMaxVel(),
                getMaxTranslationalVel(), getMaxRotVel());

        // Loop through the modules and apply commanded states.
        for (Module module : modules) {
            module.setDesiredState(moduleDesiredStates[module.moduleNumber]);
        }
    }

    /**
     * Orients the wheels in such a way that the robot can't move. Use to lock robot
     * motion after a match or when an enemy is pushing the robot.
     */
    public void lockMotion() {
        motionLocked = true;
        modules[0].setAngle(Modules.Positions.FRONT_LEFT_POS.getAngle());
        modules[1].setAngle(Modules.Positions.FRONT_RIGHT_POS.getAngle());
        modules[2].setAngle(Modules.Positions.BACK_LEFT_POS.getAngle());
        modules[3].setAngle(Modules.Positions.BACK_RIGHT_POS.getAngle());
    }

    /**
     * Unlocks wheels.
     */
    public void unlockMotion() {
        motionLocked = false;
    }

    /**
     * Sets whether the drive wheels are in brake mode or not.
     * 
     * @param brake True for brake, false for coast.
     */
    public void setWheelBrake(boolean brake) {
        for (Module module : modules) {
            module.setWheelBrake(brake);
        }
    }

    /**
     * Sets the wheels to face forwards.
     */
    public void setWheelsForward() {
        for (Module module : modules) {
            module.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
        }
    }

    public void setIndividualModule(int moduleID, SwerveModuleState desiredState) {
        modules[moduleID].setRawState(desiredState);
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

    /**
     * Gets the current pose of the robot. (0,0) is the back right corner of
     * the blue driver station/the bottom left corner of the PathPlanner app,
     * depending on your perspective. 0 degrees rotation is facing away from the
     * blue driver station, 180 degrees is facing away from the red driver station.
     * 
     * @return The robot pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the current robot velocity as a ChassisSpeeds object. Is robot relative.
     * 
     * @return The current robot velocity.
     */
    public ChassisSpeeds getRobotVelocity() {
        ChassisSpeeds robotVelocity = kinematics.toChassisSpeeds(getStates());
        return robotVelocity;
    }

    /**
     * Gets the current module positions, as an array of SwerveModulePositions.
     * 
     * @return The module positions.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (Module module : modules) {
            positions[module.moduleNumber] = module.getPosition();
        }
        return positions;
    }

    /**
     * Gets the current module states, as an array of SwerveModuleStates.
     * 
     * @return The current states.
     */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (Module module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    /**
     * Resets the current module states.
     */
    public void resetStates() {
        for (Module module : modules) {
            module.resetPosition();
        }
    }

    /**
     * A method that gets called 50 times a second to update the robot's estimated
     * position using wheel odometry.
     */
    public void updateOdometry() {
        odometryLock.lock();
        try {
            Telemetry.setRobotPose(getPose());
        } catch (Exception e) {
            odometryLock.unlock();
            throw e;
        }
        odometryLock.unlock();
        poseEstimator.update(getYaw(), getModulePositions());
        Telemetry.setRobotPose(getPose());
        Telemetry.sendField();
    }

    /**
     * A method that gets called 25 times a second to update the robot's estimated
     * position using a LimeLight that's detecting AprilTags.
     */
    public void addLLVisionMeasurement() {
        // Get pose
        Pose2d robotPose = LimelightHelpers.getBotPose2d_wpiBlue(Vision.LimeLight.APRILTAG_DETECTOR);
        // If the LimeLight returns a null pose, stop
        if (robotPose == null) {
            Telemetry.sendBoolean("LimeLight.hasTracking", false, Verbosity.LOW);
            return;
        }
        Telemetry.sendBoolean("LimeLight.hasTracking", true, Verbosity.LOW);

        // Calculate latency in seconds
        // The Limelight outputs in ms, we need it in seconds
        double limeLightLatency = (LimelightHelpers.getLatency_Capture(Vision.LimeLight.APRILTAG_DETECTOR)
                + LimelightHelpers.getLatency_Pipeline(Vision.LimeLight.APRILTAG_DETECTOR)) /
                1000.0;
        // Calculate timestamp using the current robot FPGA time and the latency.
        double captureTimeStamp = Timer.getFPGATimestamp() - limeLightLatency;

        // Call addVisionMeasurement to update the position
        addVisionMeasurement(robotPose, captureTimeStamp);
    }

    /**
     * A method that takes an estimated robot pose taken from a vision processor and
     * the timestamp in which that robot pose was taken to update estimated
     * position.
     * 
     * @param robotPose The estimated pose.
     * @param timestamp The timestamp in which the pose was measured.
     */
    public void addVisionMeasurement(Pose2d robotPose, double timestamp) {
        odometryLock.lock();
        poseEstimator.addVisionMeasurement(robotPose, timestamp);
        odometryLock.unlock();
    }

    /**
     * Resets the robot pose to a specified position.
     * 
     * @param pose The specified position.
     */
    public void resetPose(Pose2d pose) {
        odometryLock.lock();
        poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
        odometryLock.unlock();
        Rotation3d currentOffset = imu.getOffset();
        imu.setOffset(new Rotation3d(
                currentOffset.getX(),
                currentOffset.getY(),
                imu.getRawRotation3d().getZ() - pose.getRotation().getRadians()));
        Telemetry.setRobotPose(getPose());
        Telemetry.sendField();
    }

    /**
     * Sets the state of slow mode.
     * 
     * @param slowMode True for slow mode, false for full speed.
     */
    public void setSlowMode(boolean slowMode) {
        this.slowMode = slowMode;
    }

    /**
     * Resets the rotation of the robot to be forwards.
     */
    public void resetRotation() {
        resetPose(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    /**
     * Gets whether slow mode is on or not.
     * 
     * @return If slow mode is on or not. True if it is.
     */
    public boolean getSlowMode() {
        return slowMode;
    }

    /**
     * Gets the absolute max velocity of the modules. Not neccesarily the max
     * configured speed.
     * 
     * @return The max velocity, in m/s
     */
    public double getAbsoluteMaxVel() {
        double maxVel = Double.MAX_VALUE;
        for (Module module : modules) {
            if (module.getMaxVel() < maxVel) {
                maxVel = module.getMaxVel();
            }
        }
        return maxVel;
    }

    /**
     * Gets the distance of the furthest module from the center of the robot.
     * 
     * @return The radius, in meters.
     */
    public double getDrivebaseRadius() {
        double radius = 0;
        Translation2d[] modulePositions = { SwerveDrive.Modules.Positions.FRONT_LEFT_POS,
                SwerveDrive.Modules.Positions.FRONT_RIGHT_POS, SwerveDrive.Modules.Positions.BACK_LEFT_POS,
                SwerveDrive.Modules.Positions.BACK_RIGHT_POS };
        for (Translation2d position : modulePositions) {
            if (radius < position.getNorm()) {
                radius = position.getNorm();
            }
        }
        return radius;
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
     * Sends telemetry data.
     */
    public void sendTelemetry() {
        ChassisSpeeds robotVelocity = getRobotVelocity();
        Telemetry.sendNumber("SwerveDrivebase.robotVelX", robotVelocity.vxMetersPerSecond, Verbosity.MEDIUM);
        Telemetry.sendNumber("SwerveDrivebase.robotVelY", robotVelocity.vyMetersPerSecond, Verbosity.MEDIUM);
        Telemetry.sendNumber("SwerveDrivebase.robotVelRot", robotVelocity.omegaRadiansPerSecond, Verbosity.MEDIUM);
        Telemetry.sendBoolean("SwerveDrivebase.slowMode", slowMode, Verbosity.LOW);
        Telemetry.sendBoolean("SwerveDrivebase.motionLocked", motionLocked, Verbosity.LOW);
        for (Module module : modules) {
            module.sendTelemetry();
        }
    }
}
