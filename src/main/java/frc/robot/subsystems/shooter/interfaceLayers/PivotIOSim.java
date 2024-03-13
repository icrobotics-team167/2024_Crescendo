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

package frc.robot.subsystems.shooter.interfaceLayers;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;

public class PivotIOSim implements PivotIO {
  private final SingleJointedArmSim armSim;

  private final PIDController velocityPID;
  private final ArmFeedforward velocityFF;

  private final PIDController anglePID;

  public PivotIOSim() {
    armSim =
        new SingleJointedArmSim(
            DCMotor.getNeoVortex(2),
            400,
            .724,
            Meters.convertFrom(24, Inches),
            Radians.convertFrom(MIN_ANGLE, Degrees),
            Radians.convertFrom(MAX_ANGLE, Radians),
            true,
            Radians.convertFrom(
                MathUtil.interpolate(MIN_ANGLE, MAX_ANGLE, Math.random()), Degrees));

    velocityPID = new PIDController(1, 0, 0);
    velocityFF = new ArmFeedforward(0, 0.072775, 6.76);
    anglePID = new PIDController(8, 0, 0);
  }

  private enum ControlMode {
    TARGET_ANGLE,
    TARGET_VEL,
    OPEN_LOOP
  }

  private Measure<Voltage> appliedVoltage = Volts.of(0);
  private Rotation2d angleSetpoint = new Rotation2d();
  private Measure<Velocity<Angle>> velocitySetpoint = RadiansPerSecond.of(0);

  private ControlMode controlMode = ControlMode.TARGET_VEL;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    armSim.update(Robot.defaultPeriodSecs);

    inputs.angle = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.angleSetpoint = angleSetpoint;
    inputs.isTooFarDown = inputs.angle.getDegrees() <= MIN_ANGLE;
    inputs.isTooFarUp = inputs.angle.getDegrees() >= MAX_ANGLE;

    if (DriverStation.isEnabled()) {
      switch (controlMode) {
        case TARGET_ANGLE:
          velocitySetpoint =
              RadiansPerSecond.of(
                  anglePID.calculate(armSim.getAngleRads(), angleSetpoint.getRadians()));
        case TARGET_VEL:
          if ((inputs.isTooFarDown && velocitySetpoint.baseUnitMagnitude() < 0)
              || (inputs.isTooFarUp && velocitySetpoint.baseUnitMagnitude() > 0)) {
            velocitySetpoint = RadiansPerSecond.of(0);
          }
          appliedVoltage =
              Volts.of(
                  velocityPID.calculate(
                          armSim.getVelocityRadPerSec(), velocitySetpoint.in(RadiansPerSecond))
                      + velocityFF.calculate(
                          armSim.getAngleRads(), velocitySetpoint.in(RadiansPerSecond)));
        case OPEN_LOOP:
          if (DriverStation.isDisabled()) {
            appliedVoltage = Volts.of(0);
          }
          armSim.setInputVoltage(appliedVoltage.in(Volts));
      }
    }

    inputs.leaderVelocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());
    inputs.followerVelocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());
    inputs.velocitySetpoint = velocitySetpoint;

    inputs.leaderAppliedVoltage = appliedVoltage;
    inputs.followerAppliedVoltage = appliedVoltage;
    inputs.leaderAppliedCurrent = Amps.of(armSim.getCurrentDrawAmps());
    inputs.followerAppliedCurrent = Amps.of(armSim.getCurrentDrawAmps());
  }

  @Override
  public void setTargetAngle(Rotation2d targetAngle) {
    this.angleSetpoint = targetAngle;
    controlMode = ControlMode.TARGET_ANGLE;
  }

  @Override
  public void setVelocityControl(Measure<Velocity<Angle>> velocity) {
    velocitySetpoint = velocity;
    controlMode = ControlMode.TARGET_VEL;
  }

  @Override
  public void setRawControl(Measure<Voltage> voltage) {
    controlMode = ControlMode.OPEN_LOOP;
    appliedVoltage = voltage;
  }

  @Override
  public void stop() {
    setVelocityControl(RadiansPerSecond.of(0));
  }
}
