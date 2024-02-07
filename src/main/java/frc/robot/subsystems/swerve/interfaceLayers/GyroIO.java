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
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    /** If the gyro is connected or not. */
    public boolean connected = false;
    /** The yaw rotation of the robot. 0 degrees is away from the driver station, CCW+. */
    public Rotation2d yawPosition = new Rotation2d();
    /** The timestamps of the measurements captured by the async odometry thread. */
    public double[] odometryYawTimestamps = new double[] {};
    /** The yaw rotations of the robot captured by the async odometry thread. */
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    /** The yaw velocity of the robot, CCW+. */
    public Measure<Velocity<Angle>> yawVelocityRadPerSec = RadiansPerSecond.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}

  /** Sets the yaw position of the gyro. */
  public default void setYaw(Rotation2d newYaw) {}
}
