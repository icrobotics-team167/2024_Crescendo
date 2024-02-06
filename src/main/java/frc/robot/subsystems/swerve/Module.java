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
   * The rate at which module states are measured. If set too high, can cause saturation of the CAN
   * network. CAN bus usage should never go above 80%, so lower this value if Driver Station reports
   * too high of CAN utilization.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Hertz
   *       </ul>
   * </ul>
   */
  public static final double ODOMETRY_FREQUENCY = 250.0;
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
   * The gear ratio between the turn motor and the module.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Gear ratio
   *       </ul>
   * </ul>
   */
  public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
  /**
   * How many rotations the drive wheel moves when the drive motor is locked and the azimuth does
   * one full rotation. Used to compensate for gear shenanigans. Leave 0 to disable compensation.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Gear ratio
   *       </ul>
   * </ul>
   */
  // TODO: Might need to stick a minus sign in here?
  public static final double DRIVE_TURN_COMPENSATION_RATIO = 50.0 / 14.0;
  // public static final double DRIVE_TURN_COMPENSATION_RATIO = 0;
  /**
   * If the turn motor should be inverted or not. A positive control input should mean that the
   * azimuth should rotate counterclockwise, so if the turn motor needs to rotate clockwise to
   * achieve that, set this to true.
   */
  public static final boolean TURN_MOTOR_INVERTED = true;
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
  /** If the drive motor's control mode is in closed-loop velocity control, or open loop control. */
  private boolean velocityControl = true;

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

  /** Set the desired positions of the modules every robot tick. */
  public void periodic() {
    Logger.processInputs("Drive/" + getNameFromIndex(index) + " Module", inputs);

    // Run closed loop turn control
    io.setTurnPosition(angleSetpoint);

    if (velocityControl) {
      // Scale velocity based on turn error
      // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
      // towards the setpoint, its velocity should increase. This is achieved by
      // taking the component of the velocity in the direction of the setpoint.
      double adjustedSpeedSetpoint =
          speedSetpoint
              * Math.cos(inputs.turnAbsolutePosition.getRadians() - angleSetpoint.getRadians());
      io.setDriveVelocity(MetersPerSecond.of(adjustedSpeedSetpoint));
    } else {
      io.setRawDrive(speedSetpoint);
    }

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsMeters[i];
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    velocityControl = true;
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  public void runCharacterization(double voltage) {
    angleSetpoint = new Rotation2d();
    velocityControl = false;
    speedSetpoint = voltage;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.stop();
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnAbsolutePosition;
  }

  /** Returns the current drive position of the module. */
  public Measure<Distance> getPositionMeters() {
    return inputs.drivePosition;
  }

  /** Returns the current drive velocity of the module. */
  public Measure<Velocity<Distance>> getVelocityMetersPerSec() {
    return inputs.driveVelocity;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
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
