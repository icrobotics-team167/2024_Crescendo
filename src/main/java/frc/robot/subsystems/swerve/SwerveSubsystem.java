// Copyright (c) 2024 FRC 167
// https://www.thebluealliance.com/team/167
// https://github.com/icrobotics-team167
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.Driving;
import frc.robot.subsystems.swerve.interfaceLayers.GyroIO;
import frc.robot.subsystems.swerve.interfaceLayers.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.interfaceLayers.ModuleIO;
import frc.robot.subsystems.swerve.interfaceLayers.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.interfaceLayers.SparkMaxOdometryThread;
import frc.robot.subsystems.vision.VisionPoseEstimator;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
  // Constants
  /** The max linear speed of the robot. */
  public static final Measure<Velocity<Distance>> MAX_LINEAR_SPEED =
      MetersPerSecond.of(
          (Module.DRIVE_MOTOR_MAX_VEL.in(RotationsPerSecond) / Module.DRIVE_GEAR_RATIO)
              * Module.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters));
  /** The max linear acceleration of the robot. */
  public static final Measure<Velocity<Velocity<Distance>>> MAX_LINEAR_ACCELERATION =
      MetersPerSecondPerSecond.of(14);
  /** The distance between the front modules and the back modules. */
  private static final Measure<Distance> TRACK_LENGTH = Inches.of(23.5);
  /** The distance between the left modules and the right modules. */
  private static final Measure<Distance> TRACK_WIDTH = Inches.of(24.5);
  /**
   * The radius of the drivebase, as measured from the center of the robot to one of the modules.
   */
  private static final Measure<Distance> DRIVE_BASE_RADIUS =
      Meters.of(Math.hypot(TRACK_LENGTH.in(Meters) / 2.0, TRACK_WIDTH.in(Meters) / 2.0));
  /** The max angular velocity of the drivebase. */
  private static final Measure<Velocity<Angle>> MAX_ANGULAR_SPEED =
      RadiansPerSecond.of(MAX_LINEAR_SPEED.in(MetersPerSecond) / DRIVE_BASE_RADIUS.in(Meters));

  // IO layers
  /** The IO interface layer for the gyroscope. */
  private final GyroIO gyroIO;
  /** The inputs from the gyro. */
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  /** The list of modules on the drivebase. */
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  // Drive kinematics
  /** The drivebase kinematics calculator. */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  /** The raw gyro rotation. 0 degrees is away from the driver station. */
  private Rotation2d rawGyroRotation = new Rotation2d();
  /** The previously logged module positions, used to calculate deltas. */
  private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];

  // Pose estimation
  /** A thread lock to prevent read/write conflicts on odometry/pose estimation. */
  public static final Lock odometryLock = new ReentrantLock();
  /** The pose estimator, used to fuse odometry data and vision data together. */
  private SwerveDrivePoseEstimator poseEstimator;
  /** The vision-based pose estimator. */
  private VisionPoseEstimator visionPoseEstimator =
      new VisionPoseEstimator(this::addVisionMeasurement);

  /** If slowmode should be enabled or not. */
  private boolean slowmode = Driving.SLOWMODE_DEFAULT;

  public SwerveSubsystem(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    for (int i = 0; i < 4; i++) {
      lastModulePositions[i] = modules[i].getPosition();
    }
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    SparkMaxOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPoseAndGyro,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            MAX_LINEAR_SPEED.in(MetersPerSecond),
            DRIVE_BASE_RADIUS.in(Meters),
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    // Replace the default pathfinder with an AdvantageKit-compatible version
    Pathfinding.setPathfinder(new LocalADStarAK());

    // Tell PathPlanner that it should log data to AdvantageKit
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      // Log empty setpoint states when disabled
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount && i < gyroInputs.odometryYawPositions.length; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
    visionPoseEstimator.updateEstimation();
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    if (slowmode) {
      speeds.vxMetersPerSecond *= Driving.SLOWMODE_MULTIPLIER;
      speeds.vyMetersPerSecond *= Driving.SLOWMODE_MULTIPLIER;
      speeds.omegaRadiansPerSecond *= Driving.SLOWMODE_MULTIPLIER;
    }
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, speeds, MAX_LINEAR_SPEED, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    for (int i = 0; i < 4; i++) {
      modules[i].stop();
    }
  }

  /** Sets the slowmode state. */
  public void setSlowmode() {
    slowmode = !Driving.SLOWMODE_DEFAULT;
  }

  /** Resets slowmode back to its default state. */
  public void unsetSlowmode() {
    slowmode = Driving.SLOWMODE_DEFAULT;
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive velocities) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose and also sets the gyro angle. */
  public void setPoseAndGyro(Pose2d pose) {
    gyroIO.setYaw(pose.getRotation());
    setPose(pose);
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed of the drivebase. */
  public Measure<Velocity<Distance>> getMaxLinearVelocity() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular velocity of the drivebase. */
  public Measure<Velocity<Angle>> getMaxAngularVelocity() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_LENGTH.in(Meters) / 2.0, TRACK_WIDTH.in(Meters) / 2.0),
      new Translation2d(TRACK_LENGTH.in(Meters) / 2.0, -TRACK_WIDTH.in(Meters) / 2.0),
      new Translation2d(-TRACK_LENGTH.in(Meters) / 2.0, TRACK_WIDTH.in(Meters) / 2.0),
      new Translation2d(-TRACK_LENGTH.in(Meters) / 2.0, -TRACK_WIDTH.in(Meters) / 2.0)
    };
  }

  /**
   * Command factory for teleop field-oriented drive. Control input suppliers should already have
   * deadbands applied.
   */
  public Command getDriveCommand(
      DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rotInput) {
    return run(
        () -> {
          // Get control inputs and apply deadbands
          double xIn = xInput.getAsDouble();
          double yIn = yInput.getAsDouble();
          double rotIn = rotInput.getAsDouble();

          double controlMagnitude = Math.hypot(xIn, yIn);
          xIn /= Math.max(controlMagnitude, 1);
          yIn /= Math.max(controlMagnitude, 1);

          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  MAX_LINEAR_SPEED.times(xIn),
                  MAX_LINEAR_SPEED.times(yIn),
                  MAX_ANGULAR_SPEED.times(rotIn),
                  gyroInputs.yawPosition));
        });
  }

  // Set to true if you are using TalonFX motors.
  private final boolean IS_TALONFX = false;

  SysIdRoutine driveSysIDRoutine =
      new SysIdRoutine(
          new Config(
              // Adjust as needed. Quasistatic test should reach max output at the same time as the
              // timeout, IE 1 volt per second with 12 second timeout would reach max output of 12
              // volts then timeout.
              Volts.of(1).per(Second),
              Volts.of(6),
              Seconds.of(12),
              IS_TALONFX
                  ? ((state) -> Logger.recordOutput("AzimuthSysIDTestState", state.toString()))
                  : ((state) -> Logger.recordOutput("DriveSysIDTestState", state.toString()))),
          new Mechanism(this::runDriveCharacterization, null, this));
  ;

  /** Command factory for running drive system characterization. */
  public Command getDriveSysID() {
    return sequence(
        driveSysIDRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        waitSeconds(2),
        driveSysIDRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        waitSeconds(2),
        driveSysIDRoutine.dynamic(SysIdRoutine.Direction.kForward),
        waitSeconds(2),
        driveSysIDRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  SysIdRoutine azimuthSysIDRoutine =
      new SysIdRoutine(
          new Config(
              Volts.of(1).per(Second),
              Volts.of(6),
              Seconds.of(12),
              IS_TALONFX
                  ? ((state) -> Logger.recordOutput("AzimuthSysIDTestState", state.toString()))
                  : ((state) -> Logger.recordOutput("DriveSysIDTestState", state.toString()))),
          new Mechanism(this::runAzimuthCharacterization, null, this));
  ;

  /** Command factory for running azimuth system characterization. */
  public Command getAzimuthSysID() {
    return sequence(
        azimuthSysIDRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        waitSeconds(2),
        azimuthSysIDRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        waitSeconds(2),
        azimuthSysIDRoutine.dynamic(SysIdRoutine.Direction.kForward),
        waitSeconds(2),
        azimuthSysIDRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Tells the modules to run on raw voltage (or amperage) for characterization purposes.
   *
   * @param voltage The commanded voltage.
   */
  private void runDriveCharacterization(Measure<Voltage> voltage) {
    for (int i = 0; i < 4; i++) {
      modules[i].runDriveCharacterization(voltage.baseUnitMagnitude());
    }
  }

  private void runAzimuthCharacterization(Measure<Voltage> voltage) {
    for (int i = 0; i < 4; i++) {
      modules[i].runAzimuthCharacterization(voltage.baseUnitMagnitude());
    }
  }
}
