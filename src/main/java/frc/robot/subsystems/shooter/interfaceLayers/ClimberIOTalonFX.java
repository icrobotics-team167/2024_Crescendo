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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.CANConstants;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final StatusSignal<Double> leftVoltage;
  private final StatusSignal<Double> rightVoltage;
  private final StatusSignal<Double> leftCurrent;
  private final StatusSignal<Double> rightCurrent;
  private final StatusSignal<Double> leftVelocity;
  private final StatusSignal<Double> rightVelocity;
  private final StatusSignal<Double> leftPosition;
  private final StatusSignal<Double> rightPosition;

  private double MIN_ANGLE_DEGREES = -20;
  private double MAX_ANGLE_DEGREES = 75;

  public ClimberIOTalonFX() {
    leftMotor = new TalonFX(CANConstants.Shooter.CLIMBER_LEFT, CANConstants.CANIVORE_NAME);
    rightMotor = new TalonFX(CANConstants.Shooter.CLIMBER_RIGHT, CANConstants.CANIVORE_NAME);

    TalonFXConfiguration sharedConfigs = new TalonFXConfiguration();

    sharedConfigs.CurrentLimits.StatorCurrentLimit = 100;
    sharedConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    sharedConfigs.CurrentLimits.SupplyCurrentLimit = 90;
    sharedConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Napkin math, 20:1 gear ratio from motor to pulley and 52:1 ratio from pulley to arm
    sharedConfigs.Feedback.SensorToMechanismRatio = 20 * 52;

    sharedConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    sharedConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Rotations.convertFrom(MAX_ANGLE_DEGREES, Degrees);
    sharedConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    sharedConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Rotations.convertFrom(MIN_ANGLE_DEGREES, Degrees);
    leftMotor.getConfigurator().apply(sharedConfigs);
    rightMotor.getConfigurator().apply(sharedConfigs);

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

    leftVoltage = leftMotor.getMotorVoltage();
    rightVoltage = rightMotor.getMotorVoltage();
    leftCurrent = leftMotor.getStatorCurrent();
    rightCurrent = rightMotor.getStatorCurrent();
    leftVelocity = leftMotor.getVelocity();
    rightVelocity = rightMotor.getVelocity();
    leftPosition = leftMotor.getPosition();
    rightPosition = rightMotor.getPosition();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftVoltage,
        rightVoltage,
        leftCurrent,
        rightCurrent,
        leftVelocity,
        rightVelocity,
        leftPosition,
        rightPosition);

    inputs.leftAppliedVoltage = Volts.of(leftVoltage.getValueAsDouble());
    inputs.rightAppliedVoltage = Volts.of(rightVoltage.getValueAsDouble());
    inputs.leftAppliedCurrent = Amps.of(leftCurrent.getValueAsDouble());
    inputs.rightAppliedCurrent = Amps.of(rightCurrent.getValueAsDouble());
    inputs.leftAngle = Rotation2d.fromRotations(leftPosition.getValueAsDouble());
    inputs.rightAngle = Rotation2d.fromRotations(rightPosition.getValueAsDouble());
    inputs.leftVelocity = RotationsPerSecond.of(leftVelocity.getValueAsDouble());
    inputs.rightVelocity = RotationsPerSecond.of(rightVelocity.getValueAsDouble());
  }

  @Override
  public void manualControl(double control) {
    leftMotor.setVoltage(control * 12);
    rightMotor.setVoltage(control * 12);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
