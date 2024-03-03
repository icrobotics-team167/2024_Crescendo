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

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.CANConstants;
import frc.robot.util.motorUtils.SparkUtils;

public class ClimberIOSparkFlex implements ClimberIO {
  private final CANSparkFlex leftMotor;
  private final CANSparkFlex rightMotor;

  public ClimberIOSparkFlex() {
    leftMotor = new CANSparkFlex(CANConstants.Shooter.CLIMBER_LEFT, MotorType.kBrushless);
    rightMotor = new CANSparkFlex(CANConstants.Shooter.CLIMBER_RIGHT, MotorType.kBrushless);

    SparkUtils.configureSpark(leftMotor::restoreFactoryDefaults);
    SparkUtils.configureSpark(rightMotor::restoreFactoryDefaults);
    Timer.delay(0.2);
    SparkUtils.configureSpark(leftMotor::clearFaults);
    SparkUtils.configureSpark(rightMotor::clearFaults);
    SparkUtils.configureSpark(() -> leftMotor.setCANTimeout(250));
    SparkUtils.configureSpark(() -> rightMotor.setCANTimeout(250));

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

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
  }

  @Override
  public void climb() {
    leftMotor.setVoltage(12);
    rightMotor.setVoltage(12);
  }

  @Override
  public void reset() {
    leftMotor.setVoltage(-6);
    rightMotor.setVoltage(-6);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
