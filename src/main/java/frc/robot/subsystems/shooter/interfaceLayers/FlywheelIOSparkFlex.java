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
import frc.robot.util.SparkUtils;
import java.util.Set;

public class FlywheelIOSparkFlex implements FlywheelIO {
  private final CANSparkFlex topFlywheel;
  private final CANSparkFlex bottomFlywheel;

  public FlywheelIOSparkFlex() {
    topFlywheel = new CANSparkFlex(22, MotorType.kBrushless);
    bottomFlywheel = new CANSparkFlex(21, MotorType.kBrushless);

    topFlywheel.restoreFactoryDefaults();
    bottomFlywheel.restoreFactoryDefaults();
    Timer.delay(0.1);
    topFlywheel.setCANTimeout(250);
    bottomFlywheel.setCANTimeout(250);

    bottomFlywheel.setInverted(false);
    bottomFlywheel.setIdleMode(IdleMode.kCoast);
    bottomFlywheel.setSmartCurrentLimit(60);
    topFlywheel.setInverted(true);
    topFlywheel.setIdleMode(IdleMode.kCoast);
    bottomFlywheel.setSmartCurrentLimit(60);
    SparkUtils.configureFrameStrategy(
        topFlywheel,
        Set.of(
            SparkUtils.Data.POSITION,
            SparkUtils.Data.VELOCITY,
            SparkUtils.Data.INPUT,
            SparkUtils.Data.CURRENT),
        Set.of(SparkUtils.Sensor.INTEGRATED),
        false);
    SparkUtils.configureFrameStrategy(
        bottomFlywheel,
        Set.of(
            SparkUtils.Data.POSITION,
            SparkUtils.Data.VELOCITY,
            SparkUtils.Data.INPUT,
            SparkUtils.Data.CURRENT),
        Set.of(SparkUtils.Sensor.INTEGRATED),
        false);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.topPosition = Rotations.of(topFlywheel.getEncoder().getPosition());
    inputs.topVelocity = RPM.of(topFlywheel.getEncoder().getVelocity());
    inputs.topAppliedOutput = topFlywheel.getAppliedOutput();
    inputs.topAppliedCurrent = Amps.of(topFlywheel.getOutputCurrent());
    inputs.topAppliedVoltage =
        Volts.of(topFlywheel.getAppliedOutput() * topFlywheel.getBusVoltage());
    inputs.bottomPosition = Rotations.of(bottomFlywheel.getEncoder().getPosition());
    inputs.bottomVelocity = RPM.of(bottomFlywheel.getEncoder().getVelocity());
    inputs.bottomAppliedOutput = bottomFlywheel.getAppliedOutput();
    inputs.bottomAppliedCurrent = Amps.of(bottomFlywheel.getOutputCurrent());
    inputs.bottomAppliedVoltage =
        Volts.of(bottomFlywheel.getAppliedOutput() * bottomFlywheel.getBusVoltage());
  }

  @Override
  public void runSpeaker() {
    bottomFlywheel.setVoltage(12);
    topFlywheel.setVoltage(12);
  }

  @Override
  public void runAmp() {
    bottomFlywheel.setVoltage(6);
    topFlywheel.setVoltage(-6);
  }

  @Override
  public void stop() {
    topFlywheel.setVoltage(0);
    bottomFlywheel.setVoltage(0);
  }
}
