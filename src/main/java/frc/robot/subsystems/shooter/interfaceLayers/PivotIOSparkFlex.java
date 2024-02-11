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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.SparkUtils;
import java.util.Set;

public class PivotIOSparkFlex implements PivotIO {
  private DutyCycleEncoder encoder;
  private final CANSparkFlex leaderMotor;
  private final RelativeEncoder leaderEncoder;
  private final CANSparkFlex followerMotor;
  private final RelativeEncoder followerEncoder;

  private final PIDController anglePIDController;

  private final PIDController leaderPidController;
  private final ArmFeedforward leaderFFController;
  private final PIDController followerPidController;
  private final ArmFeedforward followerFFController;

  public PivotIOSparkFlex() {
    encoder = new DutyCycleEncoder(0);

    leaderMotor = new CANSparkFlex(20, MotorType.kBrushless);
    leaderEncoder = leaderMotor.getEncoder();
    SparkUtils.configureSettings(false, IdleMode.kBrake, Amps.of(60), leaderMotor);
    leaderEncoder.setPositionConversionFactor(360.0 / 400.0);
    leaderEncoder.setVelocityConversionFactor((360.0 / 400.0) / 60.0);
    SparkUtils.configureFrameStrategy(
        leaderMotor,
        Set.of(
            SparkUtils.Data.POSITION,
            SparkUtils.Data.VELOCITY,
            SparkUtils.Data.VOLTAGE,
            SparkUtils.Data.CURRENT),
        Set.of(SparkUtils.Sensor.INTEGRATED),
        false);

    followerMotor = new CANSparkFlex(19, MotorType.kBrushless);
    followerEncoder = followerMotor.getEncoder();
    SparkUtils.configureSettings(true, IdleMode.kBrake, Amps.of(60), followerMotor);
    followerEncoder.setPositionConversionFactor(360.0 / 400.0);
    followerEncoder.setVelocityConversionFactor((360.0 / 400.0) / 60.0);
    SparkUtils.configureFrameStrategy(
        followerMotor,
        Set.of(
            SparkUtils.Data.POSITION,
            SparkUtils.Data.VELOCITY,
            SparkUtils.Data.VOLTAGE,
            SparkUtils.Data.CURRENT),
        Set.of(SparkUtils.Sensor.INTEGRATED),
        false);

    anglePIDController =
        new PIDController(
            1, // Radians/sec per radian of error
            0, 0);

    leaderPidController =
        new PIDController(
            1, // Volts per degrees/sec of error
            0, 0);
    leaderFFController =
        new ArmFeedforward(
            0, // Volts to overcome static friction
            0, // Volts to overcome gravity
            0); // Volts per degrees/sec of setpoint
    followerPidController =
        new PIDController(
            1, // Volts per degrees/sec of error
            0, 0);
    followerFFController =
        new ArmFeedforward(
            0, // Volts to overcome static friction
            0, // Volts to overcome gravity
            0); // Volts per degrees/sec of setpoint
  }

  private Rotation2d targetAngle = null;
  private Measure<Velocity<Angle>> targetVelocity = RadiansPerSecond.of(0);
  private boolean angleControl = false;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.angle = getAngle();
    inputs.isTooFarDown = getAngle().getDegrees() <= PivotIO.MIN_ANGLE;
    inputs.isTooFarUp = getAngle().getDegrees() >= PivotIO.MAX_ANGLE;

    if (angleControl) {
      targetVelocity =
          RadiansPerSecond.of(
              anglePIDController.calculate(getAngle().getRadians(), targetAngle.getRadians()));
    }
    runMotor(targetVelocity);

    inputs.velocity = DegreesPerSecond.of(leaderEncoder.getVelocity());

    inputs.appliedOutput = leaderMotor.get();
    inputs.appliedVoltage = Volts.of(leaderMotor.getBusVoltage() * leaderMotor.get());
    inputs.appliedCurrent = Amps.of(leaderMotor.getOutputCurrent());
  }

  @Override
  public void setTargetAngle(Rotation2d targetAngle) {
    this.targetAngle = targetAngle;
    angleControl = true;
  }

  @Override
  public void setPivotControl(Measure<Velocity<Angle>> velocity) {
    targetVelocity = velocity;
    angleControl = false;
  }

  @Override
  public void setRawControl(Measure<Voltage> voltage) {
    leaderMotor.setVoltage(voltage.in(Volts));
    followerMotor.setVoltage(voltage.in(Volts));
  }

  @Override
  public void stop() {
    setPivotControl(RadiansPerSecond.of(0));
  }

  /** Moving average filter to smooth out the noisy input. */
  private LinearFilter angleFilter = LinearFilter.movingAverage(5);

  /** Gets the angle of the pivot mechanism. */
  private Rotation2d getAngle() {
    double rawAngle = encoder.getAbsolutePosition() - 195.0 / 360.0;
    return Rotation2d.fromRotations(angleFilter.calculate(rawAngle));
  }

  /** Run the motors at the specified pivot velocity. */
  private void runMotor(Measure<Velocity<Angle>> pivotVel) {
    leaderMotor.setVoltage(
        leaderPidController.calculate(leaderEncoder.getVelocity(), pivotVel.in(DegreesPerSecond))
            + leaderFFController.calculate(getAngle().getRadians(), pivotVel.in(RadiansPerSecond)));
    followerMotor.setVoltage(
        followerPidController.calculate(
                followerEncoder.getVelocity(), pivotVel.in(DegreesPerSecond))
            + followerFFController.calculate(
                getAngle().getRadians(), pivotVel.in(RadiansPerSecond)));
  }
}
