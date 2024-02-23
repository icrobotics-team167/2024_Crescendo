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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import frc.robot.subsystems.swerve.interfaceLayers.ModuleIO;
import frc.robot.subsystems.swerve.interfaceLayers.ModuleIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Module {
  /**
   * If set above 50 hertz and the motors are made by CTRE, then enables high frequency odometry,
   * allowing the odometry to measure position at a much higher rate, thus improving accuracy. Set
   * below 50 to disable. Does nothing if the motors are made by REV.
   *
   * <p>Setting this too high will cause issues with CAN usage. Not recommended to use if you don't
   * have a CANivore.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Hertz
   *       </ul>
   * </ul>
   */
  public static final double ODOMETRY_FREQUENCY = 50;
  // Gear ratios for SDS MK4i L2, adjust as necessary
  /**
   * The gear ratio between the drive motor and the module wheel.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Gear ratio
   *       </ul>
   * </ul>
   */
  public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  /** The max free speed of the motor. */
  public static final Measure<Velocity<Angle>> DRIVE_MOTOR_MAX_VEL = RPM.of(5800);
  /**
   * The gear ratio between the azimuth motor and the module.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Gear ratio
   *       </ul>
   * </ul>
   */
  public static final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0;
  /**
   * If the azimuth motor should be inverted or not. A positive control input should mean that the
   * azimuth should rotate counterclockwise, so if the azimuth motor needs to rotate clockwise to
   * achieve that, set this to true.
   */
  public static final boolean AZIMUTH_MOTOR_INVERTED = true;
  /** The circumference of the module wheel. */
  public static final Measure<Distance> DRIVE_WHEEL_CIRCUMFERENCE = Inches.of(4 * Math.PI);

  /** The module's IO interface, used to communicate with the actual hardware. */
  private final ModuleIO io;
  /** The data inputs from the IO interface. */
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  /**
   * The index of the module.
   *
   * <ul>
   *   <li><b>Position of module from index</b>
   *       <ul>
   *         <li>0: Front Left
   *         <li>1: Front Right
   *         <li>2: Back Left
   *         <li>3: Back Right
   *       </ul>
   * </ul>
   */
  private final int index;

  /**
   * The module's target rotation.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>{@link Rotation2d}
   *       </ul>
   * </ul>
   */
  private Rotation2d angleSetpoint = new Rotation2d();
  /**
   * The module's target velocity.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Meters per second
   *       </ul>
   * </ul>
   */
  private double speedSetpoint = 0;

  /**
   * The current states of the swerve modules.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Azimuth: {@link Rotation2d}
   *         <li>Driving: Meters per second
   *       </ul>
   * </ul>
   */
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  /**
   * Constructs a new swerve module.
   *
   * @param io The module's IO interface.
   * @param index The index of the module.
   *     <ul>
   *       <li><b>Position of module from index</b>
   *           <ul>
   *             <li>0: Front Left
   *             <li>1: Front Right
   *             <li>2: Back Left
   *             <li>3: Back Right
   *           </ul>
   *     </ul>
   */
  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    setBrakeMode(true);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  private enum DRIVE_MODE {
    CLOSEDLOOP,
    RAWDRIVE,
    RAWAZIMUTH
  }

  private DRIVE_MODE driveMode = DRIVE_MODE.CLOSEDLOOP;

  /** Set the desired positions of the modules every robot tick. */
  public void periodic() {
    Logger.processInputs("Drive/" + getNameFromIndex(index) + " Module", inputs);

    switch (driveMode) {
      case CLOSEDLOOP:
        // Scale velocity based on azimuth error
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustedSpeedSetpoint =
            speedSetpoint
                * Math.cos(
                    inputs.azimuthAbsolutePosition.getRadians() - angleSetpoint.getRadians());
        if (adjustedSpeedSetpoint < 0) {
          adjustedSpeedSetpoint = 0;
        }

        io.setDriveVelocity(MetersPerSecond.of(adjustedSpeedSetpoint));
        io.setAzimuthPosition(angleSetpoint);
        break;
      case RAWAZIMUTH:
        io.setDriveVelocity(MetersPerSecond.of(0));
        io.setRawAzimuth(angleSetpoint.getRadians());
        break;
      case RAWDRIVE:
        io.setRawDrive(speedSetpoint);
        io.setAzimuthPosition(new Rotation2d());
    }

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsMeters[i];
      Rotation2d angle = inputs.odometryAzimuthPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    driveMode = DRIVE_MODE.CLOSEDLOOP;
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  public void runDriveCharacterization(double voltage) {
    driveMode = DRIVE_MODE.RAWDRIVE;
    speedSetpoint = voltage;
  }

  public void runAzimuthCharacterization(double voltage) {
    driveMode = DRIVE_MODE.RAWAZIMUTH;
    angleSetpoint = new Rotation2d(voltage);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.stop();
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setAzimuthBrakeMode(enabled);
  }

  /** Returns the current azimuth of the module. */
  public Rotation2d getAngle() {
    return inputs.azimuthAbsolutePosition;
  }

  /** Returns the current drive position of the module. */
  public Measure<Distance> getPositionMeters() {
    return inputs.drivePosition;
  }

  /** Returns the current drive velocity of the module. */
  public Measure<Velocity<Distance>> getVelocityMetersPerSec() {
    return inputs.driveVelocity;
  }

  /** Returns the module position (azimuth and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (azimuth and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the name of a module from its index. */
  protected static String getNameFromIndex(int index) {
    switch (index) {
      case 0:
        return "Front Left";
      case 1:
        return "Front Right";
      case 2:
        return "Back Left";
      case 3:
        return "Back Right";
      default:
        throw new IndexOutOfBoundsException("Invalid module index");
    }
  }
}
