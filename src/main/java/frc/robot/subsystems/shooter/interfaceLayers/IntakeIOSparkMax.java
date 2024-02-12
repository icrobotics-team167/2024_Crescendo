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

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax motor;

  public IntakeIOSparkMax() {
    motor = new CANSparkMax(10, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isRunning = motor.get() != 0.0;
  }

  @Override
  public void run() {
    motor.set(0.75);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}