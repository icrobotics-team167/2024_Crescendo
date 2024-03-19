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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.CANConstants.Shooter;
import frc.robot.util.motorUtils.SparkUtils;
import java.util.Set;

public class FlywheelIOSparkFlex implements FlywheelIO {
  private final CANSparkFlex topFlywheel;
  private final CANSparkFlex bottomFlywheel;
  private final RelativeEncoder topFlywheelEncoder;
  private final RelativeEncoder bottomFlywheelEncoder;

  private final SimpleMotorFeedforward topFlywheelFF = new SimpleMotorFeedforward(0, 0);
  private final SimpleMotorFeedforward bottomFlywheelFF = new SimpleMotorFeedforward(0, 0);
  private final double topFlywheelP = 0;
  private final double bottomFlywheelP = 0;

  private double topSetpointRPM;
  private double bottomSetpointRPM;

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
    SparkUtils.configureSpark(() -> bottomFlywheel.setSmartCurrentLimit(80));
    topFlywheel.setInverted(false);
    SparkUtils.configureSpark(() -> topFlywheel.setIdleMode(IdleMode.kCoast));
    SparkUtils.configureSpark(() -> topFlywheel.setSmartCurrentLimit(80));
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

    // Measurement delay: 24ms
    SparkUtils.configureSpark(() -> topFlywheelEncoder.setAverageDepth(4));
    SparkUtils.configureSpark(() -> topFlywheelEncoder.setMeasurementPeriod(16));
    SparkUtils.configureSpark(() -> bottomFlywheelEncoder.setAverageDepth(4));
    SparkUtils.configureSpark(() -> bottomFlywheelEncoder.setMeasurementPeriod(16));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.topVelocitySetpoint = RPM.of(topSetpointRPM);
    inputs.bottomVelocitySetpoint = RPM.of(bottomSetpointRPM);
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
  public void runRaw(Measure<Voltage> volts) {
    topFlywheel.setVoltage(volts.in(Volts));
    bottomFlywheel.setVoltage(volts.in(Volts));
  }

  @Override
  public void runSpeaker() {
    topSetpointRPM = 2800;
    bottomSetpointRPM = 4000;

    runFlywheels(topSetpointRPM, bottomSetpointRPM);
  }

  @Override
  public void runAmp() {
    topSetpointRPM = -5000;
    bottomSetpointRPM = 4000;

    runFlywheels(topSetpointRPM, bottomSetpointRPM);
  }

  @Override
  public void runSourceIntake() {
    topSetpointRPM = -2000;
    bottomSetpointRPM = -2000;

    runFlywheels(topSetpointRPM, bottomSetpointRPM);
  }

  @Override
  public void stop() {
    topSetpointRPM = 0;
    bottomSetpointRPM = 0;
    topFlywheel.setVoltage(0);
    bottomFlywheel.setVoltage(0);
  }

  private void runFlywheels(double topSetpointRPM, double bottomSetpointRPM) {
    // No need to run the motors
    if (!isAtSetpoint(topSetpointRPM, topFlywheelEncoder.getVelocity())) {
      double topFF = topFlywheelFF.calculate(RadiansPerSecond.convertFrom(topSetpointRPM, RPM));
      double topP =
          topFlywheelP
              * RadiansPerSecond.convertFrom(
                  topSetpointRPM - topFlywheelEncoder.getVelocity(), RPM);
      topFlywheel.setVoltage(topFF + topP);
    } else {
      topFlywheel.setVoltage(0);
    }

    if (!isAtSetpoint(bottomSetpointRPM, bottomFlywheelEncoder.getVelocity())) {
      double bottomFF =
          bottomFlywheelFF.calculate(RadiansPerSecond.convertFrom(bottomSetpointRPM, RPM));
      double bottomP =
          bottomFlywheelP
              * RadiansPerSecond.convertFrom(
                  bottomSetpointRPM - bottomFlywheelEncoder.getVelocity(), RPM);
      bottomFlywheel.setVoltage(bottomFF + bottomP);
    } else {
      bottomFlywheel.setVoltage(0);
    }
  }

  private boolean isAtSetpoint(double setpoint, double measured) {
    if (setpoint == 0) {
      return false;
    }
    if (setpoint < 0) {
      return isAtSetpoint(-setpoint, -measured);
    }
    return measured >= setpoint;
  }
}
