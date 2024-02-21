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

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.Module;
import frc.robot.util.CANConstants;
import frc.robot.util.CANConstants.Drivebase;
import frc.robot.util.SwerveUtils;
import java.util.OptionalDouble;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(Drivebase.GYRO, CANConstants.CANIVORE_NAME);
  private final StatusSignal<Double> yaw = pigeon.getYaw();
  private Queue<Double> yawPositionQueue;
  private Queue<Double> yawTimestampQueue;
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

  @SuppressWarnings("unused")
  public GyroIOPigeon2(boolean phoenixDrive) {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Module.ODOMETRY_FREQUENCY > 50 ? Module.ODOMETRY_FREQUENCY : 50);
    yawVelocity.setUpdateFrequency(50);
    pigeon.optimizeBusUtilization();
    if (Module.ODOMETRY_FREQUENCY > 50) {
      if (phoenixDrive) {
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(pigeon, pigeon.getYaw());
      } else {
        yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue =
            SparkMaxOdometryThread.getInstance()
                .registerSignal(
                    () -> {
                      boolean valid = yaw.refresh().getStatus().isOK();
                      if (valid) {
                        return OptionalDouble.of(yaw.getValueAsDouble());
                      } else {
                        return OptionalDouble.empty();
                      }
                    });
      }
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = RadiansPerSecond.of(yawVelocity.getValueAsDouble());

    if (yawTimestampQueue != null) {
      inputs.odometryYawTimestamps = SwerveUtils.queueToDoubleArray(yawTimestampQueue);
      inputs.odometryYawPositions = SwerveUtils.queueToRotation2dArray(yawPositionQueue);
    }
  }

  @Override
  public void setYaw(Rotation2d newYaw) {
    pigeon.setYaw(newYaw.getDegrees());
  }
}
