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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Module;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = Robot.defaultPeriodSecs;

  private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), Module.DRIVE_GEAR_RATIO, 0.025);
  private DCMotorSim azimuthSim =
      new DCMotorSim(DCMotor.getNEO(1), Module.AZIMUTH_GEAR_RATIO, 0.004);

  private final PIDController drivePID;
  private final SimpleMotorFeedforward driveFF;

  private final PIDController azimuthPID;

  private double driveAppliedVolts = 0.0;
  private double azimuthAppliedVolts = 0.0;

  public ModuleIOSim() {
    drivePID = new PIDController(0, 0, 0);
    driveFF =
        new SimpleMotorFeedforward(0, 12 / SwerveSubsystem.MAX_LINEAR_SPEED.in(MetersPerSecond));
    azimuthPID = new PIDController(36, 0, 0);
    azimuthPID.enableContinuousInput(-.5, .5);

    azimuthSim.setState(Math.random() * 2.0 * Math.PI, 0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);
    azimuthSim.update(LOOP_PERIOD_SECS);

    inputs.drivePosition =
        Module.DRIVE_WHEEL_CIRCUMFERENCE.times(driveSim.getAngularPositionRotations());
    inputs.driveVelocity =
        Module.DRIVE_WHEEL_CIRCUMFERENCE.times(driveSim.getAngularVelocityRPM()).per(Minute);
    inputs.driveAppliedCurrent = Amps.of(driveSim.getCurrentDrawAmps());
    inputs.driveAppliedVoltage = Volts.of(driveAppliedVolts);
    inputs.driveAppliedOutput = driveAppliedVolts;

    inputs.azimuthAbsolutePosition =
        Rotation2d.fromRotations(azimuthSim.getAngularPositionRotations());
    inputs.azimuthVelocity = RPM.of(azimuthSim.getAngularVelocityRPM());
    inputs.azimuthAppliedCurrent = Amps.of(azimuthSim.getCurrentDrawAmps());
    inputs.azimuthAppliedVoltage = Volts.of(azimuthAppliedVolts);
    inputs.azimuthAppliedOutput = azimuthAppliedVolts;
  }

  @Override
  public void setDriveVelocity(Measure<Velocity<Distance>> velocity) {
    driveAppliedVolts =
        drivePID.calculate(
                driveSim.getAngularVelocityRadPerSec()
                    / (2 * Math.PI)
                    * Module.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters),
                velocity.in(MetersPerSecond))
            + driveFF.calculate(velocity.in(MetersPerSecond));
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setAzimuthPosition(Rotation2d position) {
    azimuthAppliedVolts =
        azimuthPID.calculate(azimuthSim.getAngularPositionRotations(), position.getRotations());
    azimuthSim.setInputVoltage(azimuthAppliedVolts);
  }
}
