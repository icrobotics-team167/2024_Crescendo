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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.motorUtils.SparkUtils;
import java.util.Set;

public class FeederIOSparkFlex implements FeederIO {
  private CANSparkFlex motor;

  public FeederIOSparkFlex() {
    motor = new CANSparkFlex(23, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    Timer.delay(0.1);
    motor.setInverted(true);
    motor.setCANTimeout(250);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(60);
    motor.enableVoltageCompensation(12);
    SparkUtils.configureFrameStrategy(
        motor,
        Set.of(
            SparkUtils.Data.POSITION,
            SparkUtils.Data.VELOCITY,
            SparkUtils.Data.INPUT,
            SparkUtils.Data.CURRENT),
        Set.of(SparkUtils.Sensor.INTEGRATED),
        false);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.appliedVoltage = Volts.of(motor.getBusVoltage() * motor.get());
    inputs.appliedCurrent = Amps.of(motor.getOutputCurrent());
    inputs.appliedOutput = motor.get();
    inputs.position = Rotations.of(motor.getEncoder().getPosition());
    inputs.velocity = RPM.of(motor.getEncoder().getVelocity());
  }

  @Override
  public void runForwards() {
    motor.set(1);
  }

  @Override
  public void runReverse() {
    motor.set(-0.2);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
