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

package frc.robot.subsystems.swerve.interfaceLayers;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.AutoLog;

/** The IO Interface for swerve modules. */
public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    /** The current distance that the module has driven so far. */
    public Measure<Distance> drivePosition = Meters.of(0);
    /** The current drive velocity of the module. */
    public Measure<Velocity<Distance>> driveVelocity = MetersPerSecond.of(0);
    /** The voltage applied to the motor by the motor controller. */
    public Measure<Voltage> driveAppliedVoltage = Volts.of(0);
    /** The total output applied to the motor by the closed loop control. */
    public double driveAppliedOutput = 0.0;

    public double[] driveAppliedCurrentAmps = new double[] {};

    public Rotation2d azimuthAbsolutePosition = new Rotation2d();
    public Measure<Velocity<Angle>> azimuthVelocity = RadiansPerSecond.of(0);
    public Measure<Voltage> azimuthAppliedVoltage = Volts.of(0);
    public double azimuthAppliedOutput = 0.0;
    public double[] azimuthAppliedCurrentAmps = new double[] {};

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsMeters = new double[] {};
    public Rotation2d[] odometryAzimuthPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /**
   * Sets the target velocity of the drive motor's closed-loop control.
   *
   * @param velocity The target velocity.
   */
  public default void setDriveVelocity(Measure<Velocity<Distance>> velocity) {}

  /**
   * Sets the raw open-loop output of the drive motor for system characterization purposes.
   *
   * <p>NOTE: Normally the units here would be volts, but when using TorqueControlFOC on TalonFX
   * modules, the units are in amps.
   *
   * @param rawUnits The raw output, in the units used by the control scheme.
   * @see SwerveSubsystem#getSysID
   */
  public default void setRawDrive(double rawUnits) {}

  /**
   * Sets the target position of the azimuth's closed-loop control.
   *
   * @param position The target position.
   */
  public default void setAzimuthPosition(Rotation2d position) {}

  /** Stops all motor control input. */
  public default void stop() {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the azimuth motor. */
  public default void setAzimuthBrakeMode(boolean enable) {}

  public default void configureDriveSysID() {}

  public default void configureAzimuthSysID() {}
}
