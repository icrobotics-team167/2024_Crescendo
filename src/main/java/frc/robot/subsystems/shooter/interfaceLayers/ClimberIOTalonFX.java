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
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.CANConstants;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final DutyCycleEncoder leftEncoder;
  private final DutyCycleEncoder rightEncoder;
  private final CurrentLimitsConfigs configs = new CurrentLimitsConfigs();

  private final StatusSignal<Double> leftVoltage;
  private final StatusSignal<Double> rightVoltage;
  private final StatusSignal<Double> leftCurrent;
  private final StatusSignal<Double> rightCurrent;
  private final StatusSignal<Double> leftVelocity;
  private final StatusSignal<Double> rightVelocity;

  private double MIN_ANGLE_DEGREES = -10;
  private double MAX_ANGLE_DEGREES = 90;

  private PIDController leftPIDs = new PIDController(0, 0, 0);
  private PIDController rightPIDs = new PIDController(0, 0, 0);

  private double left_kG = 0;
  private double right_kG = 0;

  public ClimberIOTalonFX() {
    leftMotor = new TalonFX(CANConstants.Shooter.CLIMBER_LEFT, CANConstants.CANIVORE_NAME);
    rightMotor = new TalonFX(CANConstants.Shooter.CLIMBER_RIGHT, CANConstants.CANIVORE_NAME);
    leftEncoder = new DutyCycleEncoder(1);
    rightEncoder = new DutyCycleEncoder(3);

    configs.StatorCurrentLimit = 100;
    configs.StatorCurrentLimitEnable = true;
    configs.SupplyCurrentLimit = 90;
    configs.SupplyCurrentLimitEnable = true;
    leftMotor.getConfigurator().apply(configs);
    rightMotor.getConfigurator().apply(configs);

    MotorOutputConfigs leftOutputConfigs = new MotorOutputConfigs();
    MotorOutputConfigs rightOutputConfigs = new MotorOutputConfigs();
    leftOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    rightOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    leftMotor.getConfigurator().apply(leftOutputConfigs);
    rightMotor.getConfigurator().apply(rightOutputConfigs);

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

    leftVoltage = leftMotor.getMotorVoltage();
    rightVoltage = rightMotor.getMotorVoltage();
    leftCurrent = leftMotor.getStatorCurrent();
    rightCurrent = rightMotor.getStatorCurrent();
    leftVelocity = leftMotor.getVelocity();
    rightVelocity = rightMotor.getVelocity();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftVoltage, rightVoltage, leftCurrent, rightCurrent, leftVelocity, rightVelocity);

    inputs.leftAppliedVoltage = Volts.of(leftVoltage.getValueAsDouble());
    inputs.rightAppliedVoltage = Volts.of(rightVoltage.getValueAsDouble());
    inputs.leftAppliedCurrent = Amps.of(leftCurrent.getValueAsDouble());
    inputs.rightAppliedCurrent = Amps.of(rightCurrent.getValueAsDouble());
    inputs.leftAngle = getLeftAngle();
    inputs.rightAngle = getRightAngle();
    inputs.leftVelocity = RPM.of(leftVelocity.getValueAsDouble());
    inputs.rightVelocity = RPM.of(rightVelocity.getValueAsDouble());
  }

  @Override
  public void manualControl(double control) {
    control = MathUtil.clamp(control, 0, 1);
    Rotation2d angleSetpoint =
        Rotation2d.fromDegrees(
            control * (MAX_ANGLE_DEGREES - MIN_ANGLE_DEGREES) - MIN_ANGLE_DEGREES);
    Rotation2d leftAngle = getLeftAngle();
    Rotation2d rightAngle = getRightAngle();

    double leftOutput = leftPIDs.calculate(leftAngle.getDegrees(), angleSetpoint.getDegrees());
    double rightOutput = rightPIDs.calculate(rightAngle.getDegrees(), angleSetpoint.getDegrees());

    if (leftOutput < 0 && isTooLow(leftAngle.getDegrees())) {
      leftOutput = 0;
    }
    if (leftOutput > 0 && isTooHigh(leftAngle.getDegrees())) {
      leftOutput = 0;
    }
    if (rightOutput < 0 && isTooLow(rightAngle.getDegrees())) {
      rightOutput = 0;
    }
    if (rightOutput > 0 && isTooHigh(rightAngle.getDegrees())) {
      rightOutput = 0;
    }

    if (angleSetpoint.getDegrees() != MIN_ANGLE_DEGREES) {
      leftOutput +=
          left_kG
              * Math.cos(
                  angleSetpoint.getRadians() + Radians.convertFrom(MIN_ANGLE_DEGREES, Degrees));
      rightOutput +=
          right_kG
              * Math.cos(
                  angleSetpoint.getDegrees() + Radians.convertFrom(MIN_ANGLE_DEGREES, Degrees));
    }

    leftMotor.setVoltage(leftOutput);
    rightMotor.setVoltage(rightOutput);
  }

  private boolean isTooLow(double angleDegrees) {
    // return false;
    return angleDegrees < MIN_ANGLE_DEGREES && angleDegrees - MIN_ANGLE_DEGREES >= 0;
  }

  private boolean isTooHigh(double angleDegrees) {
    // return false;
    return angleDegrees > MAX_ANGLE_DEGREES;
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  private LinearFilter leftAngleFilter = LinearFilter.movingAverage(4);

  private Rotation2d getLeftAngle() {
    double rawAngle = leftEncoder.getAbsolutePosition() - (-76.2 / 360.0);
    return Rotation2d.fromRadians(
        MathUtil.angleModulus(Units.rotationsToRadians(leftAngleFilter.calculate(rawAngle))));
  }

  private LinearFilter rightAngleFilter = LinearFilter.movingAverage(4);

  // 0: -162.5
  // 90: 106.1
  // going up makes positive
  private Rotation2d getRightAngle() {
    double rawAngle = rightEncoder.getAbsolutePosition() - (-163.5 / 360.0);
    return Rotation2d.fromRadians(
        MathUtil.angleModulus(Units.rotationsToRadians(rightAngleFilter.calculate(rawAngle))));
  }
}
