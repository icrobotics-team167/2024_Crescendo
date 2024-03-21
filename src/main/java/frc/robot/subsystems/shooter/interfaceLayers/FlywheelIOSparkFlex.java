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
  private final CANSparkFlex guideWheel;
  private final RelativeEncoder topFlywheelEncoder;
  private final RelativeEncoder bottomFlywheelEncoder;
  private final RelativeEncoder guideWheelEncoder;

  private double topSetpointRPM;
  private double bottomSetpointRPM;
  private double guideWheelSetpointRPM;

  public FlywheelIOSparkFlex() {
    topFlywheel = new CANSparkFlex(Shooter.TOP_FLYWHEEL, MotorType.kBrushless);
    bottomFlywheel = new CANSparkFlex(Shooter.BOTTOM_FLYWHEEL, MotorType.kBrushless);
    guideWheel = new CANSparkFlex(Shooter.GUIDE_WHEEL, MotorType.kBrushless);

    SparkUtils.configureSpark(() -> topFlywheel.restoreFactoryDefaults());
    SparkUtils.configureSpark(() -> bottomFlywheel.restoreFactoryDefaults());
    SparkUtils.configureSpark(() -> guideWheel.restoreFactoryDefaults());
    Timer.delay(0.1);
    SparkUtils.configureSpark(() -> topFlywheel.clearFaults());
    SparkUtils.configureSpark(() -> bottomFlywheel.clearFaults());
    SparkUtils.configureSpark(() -> guideWheel.clearFaults());
    SparkUtils.configureSpark(() -> topFlywheel.setCANTimeout(250));
    SparkUtils.configureSpark(() -> bottomFlywheel.setCANTimeout(250));
    SparkUtils.configureSpark(() -> guideWheel.setCANTimeout(250));

    bottomFlywheel.setInverted(true);
    SparkUtils.configureSpark(() -> bottomFlywheel.setIdleMode(IdleMode.kCoast));
    SparkUtils.configureSpark(() -> bottomFlywheel.setSmartCurrentLimit(80));
    topFlywheel.setInverted(false);
    SparkUtils.configureSpark(() -> topFlywheel.setIdleMode(IdleMode.kCoast));
    SparkUtils.configureSpark(() -> topFlywheel.setSmartCurrentLimit(80));
    guideWheel.setInverted(false);
    SparkUtils.configureSpark(() -> guideWheel.setIdleMode(IdleMode.kCoast));
    SparkUtils.configureSpark(() -> guideWheel.setSmartCurrentLimit(80));
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
    SparkUtils.configureFrameStrategy(
        guideWheel,
        Set.of(
            SparkUtils.Data.POSITION,
            SparkUtils.Data.VELOCITY,
            SparkUtils.Data.INPUT,
            SparkUtils.Data.CURRENT),
        Set.of(SparkUtils.Sensor.INTEGRATED),
        false);

    topFlywheelEncoder = topFlywheel.getEncoder();
    bottomFlywheelEncoder = bottomFlywheel.getEncoder();
    guideWheelEncoder = guideWheel.getEncoder();

    SparkUtils.configureSpark(() -> topFlywheelEncoder.setAverageDepth(4));
    SparkUtils.configureSpark(() -> topFlywheelEncoder.setMeasurementPeriod(16));
    SparkUtils.configureSpark(() -> bottomFlywheelEncoder.setAverageDepth(4));
    SparkUtils.configureSpark(() -> bottomFlywheelEncoder.setMeasurementPeriod(16));
    SparkUtils.configureSpark(() -> guideWheelEncoder.setAverageDepth(4));
    SparkUtils.configureSpark(() -> guideWheelEncoder.setMeasurementPeriod(16));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.topVelocitySetpoint = RPM.of(topSetpointRPM);
    inputs.bottomVelocitySetpoint = RPM.of(bottomSetpointRPM);
    inputs.guideWheelSetpoint = RPM.of(guideWheelSetpointRPM);
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
    inputs.guidePosition = Rotations.of(guideWheelEncoder.getPosition());
    inputs.guideVelocity = RPM.of(guideWheelEncoder.getVelocity());
    inputs.guideAppliedOutput = guideWheel.getAppliedOutput();
    inputs.guideAppliedCurrent = Amps.of(guideWheel.getOutputCurrent());
    inputs.guideAppliedVoltage =
        Volts.of(guideWheel.getAppliedOutput() * guideWheel.getBusVoltage());
  }

  @Override
  public void runSpeaker() {
    topSetpointRPM = 3250;
    bottomSetpointRPM = 4000;
    guideWheelSetpointRPM = 4000;

    if (topFlywheelEncoder.getVelocity() < topSetpointRPM) {
      topFlywheel.setVoltage(12);
    } else {
      topFlywheel.setVoltage(0);
    }

    if (bottomFlywheelEncoder.getVelocity() < bottomSetpointRPM) {
      bottomFlywheel.setVoltage(12);
    } else {
      bottomFlywheel.setVoltage(0);
    }

    if (guideWheelEncoder.getVelocity() < guideWheelSetpointRPM) {
      guideWheel.setVoltage(12);
    } else {
      guideWheel.setVoltage(0);
    }
  }

  @Override
  public void runAmp() {
    topSetpointRPM = -4500;
    bottomSetpointRPM = 1;
    guideWheelSetpointRPM = 4000;

    if (topFlywheelEncoder.getVelocity() > topSetpointRPM) {
      topFlywheel.setVoltage(-12);
    } else {
      topFlywheel.setVoltage(0);
    }

    // if (bottomFlywheelEncoder.getVelocity() < bottomSetpointRPM) {
    //   bottomFlywheel.setVoltage(12);
    // } else {
    //   bottomFlywheel.setVoltage(0);
    // }

    if (guideWheelEncoder.getVelocity() < guideWheelSetpointRPM) {
      guideWheel.setVoltage(12);
    } else {
      guideWheel.setVoltage(0);
    }
  }

  @Override
  public void runSourceIntake() {
    topSetpointRPM = -2000;
    bottomSetpointRPM = -2000;
    guideWheelSetpointRPM = -2000;

    if (topFlywheelEncoder.getVelocity() > topSetpointRPM) {
      topFlywheel.setVoltage(-12);
    } else {
      topFlywheel.setVoltage(0);
    }

    if (bottomFlywheelEncoder.getVelocity() > bottomSetpointRPM) {
      bottomFlywheel.setVoltage(-12);
    } else {
      bottomFlywheel.setVoltage(0);
    }

    if (guideWheelEncoder.getVelocity() > guideWheelSetpointRPM) {
      guideWheel.setVoltage(-12);
    } else {
      guideWheel.setVoltage(0);
    }
  }

  @Override
  public void stop() {
    topSetpointRPM = 0;
    bottomSetpointRPM = 0;
    guideWheelSetpointRPM = 0;
    topFlywheel.setVoltage(0);
    bottomFlywheel.setVoltage(0);
    guideWheel.setVoltage(0);
  }
}
