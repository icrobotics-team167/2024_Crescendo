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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and azimuth motors, with the absolute position
 * initialized to a random value. The flywheel sims are not physically accurate, but provide a
 * decent approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private FlywheelSim driveMotorSim;
  private PIDController drivePID = new PIDController(1.0 / 4.5, 0, 0);

  private FlywheelSim azimuthMotorSim;
  private PIDController azimuthPID = new PIDController(1.0, 0, 0);

  private Rotation2d azimuthAbsolutePosition =
      new Rotation2d((Math.random() * 2.0 - 1.0) * Math.PI);

  public ModuleIOSim() {
    driveMotorSim = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 6.75, 0.025);
    azimuthMotorSim = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 150.0 / 75.0, 0.004);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveAppliedOutput =
        MathUtil.clamp(
            drivePID.calculate(
                getDistancePerRadian(driveMotorSim.getAngularVelocityRadPerSec())
                    .per(Second)
                    .in(MetersPerSecond)),
            -1,
            1);
    inputs.driveAppliedCurrent = Amps.of(driveMotorSim.getCurrentDrawAmps());

    inputs.azimuthAppliedOutput =
        MathUtil.clamp(azimuthPID.calculate(inputs.azimuthAbsolutePosition.getRotations()), -1, 1);
    inputs.azimuthAppliedCurrent = Amps.of(azimuthMotorSim.getCurrentDrawAmps());

    driveMotorSim.setInputVoltage(inputs.driveAppliedOutput * 12);
    azimuthMotorSim.setInputVoltage(inputs.azimuthAppliedOutput * 12);

    driveMotorSim.update(Robot.defaultPeriodSecs);
    azimuthMotorSim.update(Robot.defaultPeriodSecs);

    inputs.driveVelocity =
        getDistancePerRadian(driveMotorSim.getAngularVelocityRadPerSec()).per(Second);
    double driveAngleDiffRad =
        azimuthMotorSim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs;
    inputs.drivePosition.plus(getDistancePerRadian(driveAngleDiffRad));

    inputs.azimuthVelocity = RadiansPerSecond.of(azimuthMotorSim.getAngularVelocityRadPerSec());
    double azimuthDeltaRad =
        azimuthMotorSim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs;
    azimuthAbsolutePosition =
        new Rotation2d(
            MathUtil.inputModulus(
                azimuthAbsolutePosition.getRadians() + azimuthDeltaRad, -Math.PI, Math.PI));
    inputs.azimuthAbsolutePosition = azimuthAbsolutePosition;

    inputs.odometryDrivePositionsMeters = new double[] {inputs.drivePosition.in(Meters)};
    inputs.odometryAzimuthPositions = new Rotation2d[] {inputs.azimuthAbsolutePosition};
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
  }

  @Override
  public void setDriveVelocity(Measure<Velocity<Distance>> velocity) {
    drivePID.setSetpoint(velocity.in(MetersPerSecond));
  }

  @Override
  public void setAzimuthPosition(Rotation2d position) {
    azimuthPID.setSetpoint(position.getRotations());
  }

  private Measure<Distance> getDistancePerRadian(double radians) {
    return Inches.of(4 * Math.PI * radians);
  }
}
