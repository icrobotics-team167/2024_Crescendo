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
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.CANConstants.Shooter;
import frc.robot.util.motorUtils.SparkUtils;
import java.util.Set;

public class FlywheelIOSparkFlex implements FlywheelIO {
  private final CANSparkFlex topFlywheel;
  private final CANSparkFlex bottomFlywheel;
  private final RelativeEncoder topFlywheelEncoder;
  private final RelativeEncoder bottomFlywheelEncoder;

  private double setpoint;

  public FlywheelIOSparkFlex() {
    topFlywheel = new CANSparkFlex(Shooter.TOP_FLYWHEEL, MotorType.kBrushless);
    bottomFlywheel = new CANSparkFlex(Shooter.BOTTOM_FLYWHEEL, MotorType.kBrushless);

    SparkUtils.configureSpark(() -> topFlywheel.restoreFactoryDefaults());
    SparkUtils.configureSpark(() -> bottomFlywheel.restoreFactoryDefaults());
    Timer.delay(0.1);
    SparkUtils.configureSpark(() -> topFlywheel.clearFaults());
    SparkUtils.configureSpark(() -> bottomFlywheel.clearFaults());
    SparkUtils.configureSpark(() -> topFlywheel.setCANTimeout(250));
    SparkUtils.configureSpark(() -> bottomFlywheel.setCANTimeout(250));

    bottomFlywheel.setInverted(false);
    SparkUtils.configureSpark(() -> bottomFlywheel.setIdleMode(IdleMode.kCoast));
    SparkUtils.configureSpark(() -> bottomFlywheel.setSmartCurrentLimit(60));
    topFlywheel.setInverted(false);
    SparkUtils.configureSpark(() -> topFlywheel.setIdleMode(IdleMode.kCoast));
    SparkUtils.configureSpark(() -> topFlywheel.setSmartCurrentLimit(60));
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

    topFlywheelEncoder = topFlywheel.getEncoder();
    bottomFlywheelEncoder = bottomFlywheel.getEncoder();

    SparkUtils.configureSpark(() -> topFlywheelEncoder.setAverageDepth(4));
    SparkUtils.configureSpark(() -> topFlywheelEncoder.setMeasurementPeriod(16));
    SparkUtils.configureSpark(() -> bottomFlywheelEncoder.setAverageDepth(4));
    SparkUtils.configureSpark(() -> bottomFlywheelEncoder.setMeasurementPeriod(16));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocitySetpointRPM = setpoint;
    inputs.topPosition = Rotations.of(topFlywheelEncoder.getPosition());
    inputs.topVelocity = RPM.of(topFlywheelEncoder.getVelocity());
    inputs.topAppliedOutput = topFlywheel.getAppliedOutput();
    inputs.topAppliedCurrent = Amps.of(topFlywheel.getOutputCurrent());
    inputs.topAppliedVoltage =
        Volts.of(topFlywheel.getAppliedOutput() * topFlywheel.getBusVoltage());
    inputs.bottomPosition = Rotations.of(bottomFlywheelEncoder.getPosition());
    inputs.bottomVelocity = RPM.of(bottomFlywheelEncoder.getVelocity());
    inputs.bottomAppliedOutput = bottomFlywheel.getAppliedOutput();
    inputs.bottomAppliedCurrent = Amps.of(bottomFlywheel.getOutputCurrent());
    inputs.bottomAppliedVoltage =
        Volts.of(bottomFlywheel.getAppliedOutput() * bottomFlywheel.getBusVoltage());
  }

  @Override
  public void runSpeaker() {
    setpoint = 4000;
    if (bottomFlywheelEncoder.getVelocity() < setpoint) {
      bottomFlywheel.setVoltage(12);
    } else {
      bottomFlywheel.setVoltage(0);
    }

    if (Math.abs(topFlywheelEncoder.getVelocity()) < (setpoint * .7)) {
      topFlywheel.setVoltage(12);
    } else {
      topFlywheel.setVoltage(0);
    }
  }

  @Override
  public void runAmp() {
    setpoint = 5000;
    if (bottomFlywheelEncoder.getVelocity() < setpoint) {
      bottomFlywheel.setVoltage(12);
    } else {
      bottomFlywheel.setVoltage(0);
    }

    if (Math.abs(topFlywheelEncoder.getVelocity()) < setpoint * 1.2) {
      topFlywheel.setVoltage(-12);
    } else {
      topFlywheel.setVoltage(0);
    }
  }

  @Override
  public void runSourceIntake() {
    setpoint = 2000;
    if (Math.abs(bottomFlywheelEncoder.getVelocity()) < setpoint) {
      bottomFlywheel.setVoltage(-12);
    } else {
      bottomFlywheel.setVoltage(0);
    }
    if (Math.abs(topFlywheelEncoder.getVelocity()) < setpoint) {
      topFlywheel.setVoltage(-12);
    } else {
      topFlywheel.setVoltage(0);
    }
  }

  @Override
  public void stop() {
    setpoint = 0;
    topFlywheel.setVoltage(0);
    bottomFlywheel.setVoltage(0);
  }
}
