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

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.CANConstants;
import frc.robot.util.motorUtils.SparkUtils;

public class ClimberIOSparkFlex implements ClimberIO {
  private final CANSparkFlex leftMotor;
  private final CANSparkFlex rightMotor;
  private final DutyCycleEncoder leftEncoder;
  private final DutyCycleEncoder rightEncoder;

  private double MIN_ANGLE = -10;
  private double MAX_ANGLE = 90;

  // Left: 24.285 degrees per second per volt
  // Right: 23.333 degrees per second per volt
  private double fudgeFactor = .906976;

  public ClimberIOSparkFlex() {
    leftMotor = new CANSparkFlex(CANConstants.Shooter.CLIMBER_LEFT, MotorType.kBrushless);
    rightMotor = new CANSparkFlex(CANConstants.Shooter.CLIMBER_RIGHT, MotorType.kBrushless);
    leftEncoder = new DutyCycleEncoder(1);
    rightEncoder = new DutyCycleEncoder(3);

    SparkUtils.configureSpark(leftMotor::restoreFactoryDefaults);
    SparkUtils.configureSpark(rightMotor::restoreFactoryDefaults);
    Timer.delay(0.2);
    SparkUtils.configureSpark(leftMotor::clearFaults);
    SparkUtils.configureSpark(rightMotor::clearFaults);
    SparkUtils.configureSpark(() -> leftMotor.setCANTimeout(250));
    SparkUtils.configureSpark(() -> rightMotor.setCANTimeout(250));

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
    SparkUtils.configureSpark(() -> leftMotor.setIdleMode(IdleMode.kBrake));
    SparkUtils.configureSpark(() -> rightMotor.setIdleMode(IdleMode.kBrake));

    SparkUtils.configureSpark(() -> leftMotor.setSmartCurrentLimit(80));
    SparkUtils.configureSpark(() -> leftMotor.setSecondaryCurrentLimit(100));
    SparkUtils.configureSpark(() -> rightMotor.setSmartCurrentLimit(80));
    SparkUtils.configureSpark(() -> rightMotor.setSecondaryCurrentLimit(100));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftAppliedVoltage = Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
    inputs.rightAppliedVoltage =
        Volts.of(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
    inputs.leftAppliedCurrent = Amps.of(leftMotor.getOutputCurrent());
    inputs.rightAppliedCurrent = Amps.of(rightMotor.getOutputCurrent());
    inputs.leftAngle = getLeftAngle();
    inputs.rightAngle = getRightAngle();
    inputs.leftVelocity = RPM.of(leftMotor.getEncoder().getVelocity());
    inputs.rightVelocity = RPM.of(rightMotor.getEncoder().getVelocity());
  }

  @Override
  public void climb() {
    manualControl(1);
  }

  @Override
  public void raise() {
    manualControl(-1);
  }

  @Override
  public void manualControl(double control) {
    if ((isTooLow(getLeftAngle().getDegrees()) && control < 0)
        || (isTooHigh(getLeftAngle().getDegrees()) && control > 0)) {
      leftMotor.stopMotor();
    } else {
      leftMotor.setVoltage(control * 12 * fudgeFactor);
    }

    if ((isTooLow(getRightAngle().getDegrees()) && control < 0)
        || (isTooHigh(getRightAngle().getDegrees()) && control > 0)) {
      rightMotor.stopMotor();
    } else {
      rightMotor.setVoltage(control * 12);
    }
  }

  private boolean isTooLow(double angleDegrees) {
    return angleDegrees < MIN_ANGLE && angleDegrees - MIN_ANGLE >= 0;
  }

  private boolean isTooHigh(double angleDegrees) {
    return angleDegrees > MAX_ANGLE;
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  private LinearFilter leftAngleFilter = LinearFilter.movingAverage(4);

  private Rotation2d getLeftAngle() {
    double rawAngle = leftEncoder.getAbsolutePosition() - (167.5 / 360.0);
    return Rotation2d.fromRadians(
        MathUtil.angleModulus(Units.rotationsToRadians(leftAngleFilter.calculate(rawAngle))));
  }

  private LinearFilter rightAngleFilter = LinearFilter.movingAverage(4);

  private Rotation2d getRightAngle() {
    double rawAngle = (201.0 / 360.0) - rightEncoder.getAbsolutePosition();
    return Rotation2d.fromRadians(
        MathUtil.angleModulus(Units.rotationsToRadians(rightAngleFilter.calculate(rawAngle))));
  }
}
