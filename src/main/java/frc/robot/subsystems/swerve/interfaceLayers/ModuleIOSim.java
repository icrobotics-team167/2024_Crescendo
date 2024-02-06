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
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private FlywheelSim driveMotorSim;
  private PIDController drivePID = new PIDController(1.0 / 4.5, 0, 0);

  private FlywheelSim turnMotorSim;
  private PIDController turnPID = new PIDController(1.0, 0, 0);

  private Rotation2d turnAbsolutePosition = new Rotation2d((Math.random() * 2.0 - 1.0) * Math.PI);

  public ModuleIOSim() {
    driveMotorSim = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 6.75, 0.025);
    turnMotorSim = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 150.0 / 75.0, 0.004);
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
    inputs.driveAppliedVoltage = Volts.of(inputs.driveAppliedOutput * 12);
    inputs.driveAppliedCurrentAmps = new double[] {driveMotorSim.getCurrentDrawAmps()};

    inputs.turnAppliedOutput =
        MathUtil.clamp(turnPID.calculate(inputs.turnAbsolutePosition.getRotations()), -1, 1);
    inputs.turnAppliedVoltage = Volts.of(inputs.turnAppliedOutput * 12);
    inputs.turnAppliedCurrentAmps = new double[] {turnMotorSim.getCurrentDrawAmps()};

    driveMotorSim.setInputVoltage(inputs.driveAppliedVoltage.in(Volts));
    turnMotorSim.setInputVoltage(inputs.turnAppliedVoltage.in(Volts));

    driveMotorSim.update(Robot.defaultPeriodSecs);
    turnMotorSim.update(Robot.defaultPeriodSecs);

    inputs.driveVelocity =
        getDistancePerRadian(driveMotorSim.getAngularVelocityRadPerSec()).per(Second);
    double driveAngleDiffRad = turnMotorSim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs;
    inputs.drivePosition.plus(getDistancePerRadian(driveAngleDiffRad));

    inputs.turnVelocity = RadiansPerSecond.of(turnMotorSim.getAngularVelocityRadPerSec());
    double turnAngleDiffRad = turnMotorSim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs;
    turnAbsolutePosition =
        new Rotation2d(
            MathUtil.inputModulus(
                turnAbsolutePosition.getRadians() + turnAngleDiffRad, -Math.PI, Math.PI));
    inputs.turnAbsolutePosition = turnAbsolutePosition;

    inputs.odometryDrivePositionsMeters = new double[] {inputs.drivePosition.in(Meters)};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnAbsolutePosition};
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
  }

  @Override
  public void setDriveVelocity(Measure<Velocity<Distance>> velocity) {
    drivePID.setSetpoint(velocity.in(MetersPerSecond));
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    turnPID.setSetpoint(position.getRotations());
  }

  private Measure<Distance> getDistancePerRadian(double radians) {
    return Inches.of(4 * Math.PI * radians);
  }
}
