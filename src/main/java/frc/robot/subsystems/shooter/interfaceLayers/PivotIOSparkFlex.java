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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.util.CANConstants.Shooter;
import frc.robot.util.motorUtils.SparkUtils;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class PivotIOSparkFlex implements PivotIO {
  private final DutyCycleEncoder encoder;
  private final CANSparkFlex leaderMotor;
  private final RelativeEncoder leaderEncoder;
  private final CANSparkFlex followerMotor;
  private final RelativeEncoder followerEncoder;

  private final TrapezoidProfile angleMotionProfile;

  private final PIDController leaderPidController;
  private final ArmFeedforward leaderFFController;
  private final PIDController followerPidController;
  private final ArmFeedforward followerFFController;

  private double leaderSetpoint = 0;
  private double followerSetpoint = 0;

  public PivotIOSparkFlex() {
    encoder = new DutyCycleEncoder(0);

    leaderMotor = new CANSparkFlex(Shooter.PIVOT_LEADER, MotorType.kBrushless);
    followerMotor = new CANSparkFlex(Shooter.PIVOT_FOLLOWER, MotorType.kBrushless);
    leaderMotor.restoreFactoryDefaults();
    followerMotor.restoreFactoryDefaults();
    Timer.delay(0.1);

    leaderMotor.setCANTimeout(250);
    followerMotor.setCANTimeout(250);

    leaderEncoder = leaderMotor.getEncoder();
    leaderMotor.setIdleMode(IdleMode.kBrake);
    leaderMotor.setInverted(true);
    leaderMotor.setSmartCurrentLimit(60);
    leaderEncoder.setPositionConversionFactor(360.0 / 400.0);
    leaderEncoder.setVelocityConversionFactor((360.0 / 400.0) / 60.0);
    SparkUtils.configureFrameStrategy(
        leaderMotor,
        Set.of(
            SparkUtils.Data.POSITION,
            SparkUtils.Data.VELOCITY,
            SparkUtils.Data.INPUT,
            SparkUtils.Data.CURRENT),
        Set.of(SparkUtils.Sensor.INTEGRATED),
        false);

    followerEncoder = followerMotor.getEncoder();
    followerMotor.setInverted(false);
    followerMotor.setIdleMode(IdleMode.kBrake);
    followerMotor.setSmartCurrentLimit(60);
    followerEncoder.setPositionConversionFactor(360.0 / 400.0);
    followerEncoder.setVelocityConversionFactor((360.0 / 400.0) / 60.0);
    SparkUtils.configureFrameStrategy(
        followerMotor,
        Set.of(
            SparkUtils.Data.POSITION,
            SparkUtils.Data.VELOCITY,
            SparkUtils.Data.INPUT,
            SparkUtils.Data.CURRENT),
        Set.of(SparkUtils.Sensor.INTEGRATED),
        false);

    angleMotionProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                DegreesPerSecond.of(15), DegreesPerSecond.per(Second).of(30)));

    leaderPidController =
        new PIDController(
            1.0 / 20, // Volts per degrees/sec of error
            0, 0);
    leaderFFController =
        new ArmFeedforward(
            0.05, // Volts to overcome static friction
            0, // Volts to overcome gravity
            10.5 / (Math.PI / 2)); // Volts per radians/sec of setpoint
    followerPidController =
        new PIDController(
            1.0 / 20, // Volts per degrees/sec of error
            0, 0);
    followerFFController =
        new ArmFeedforward(
            0.05, // Volts to overcome static friction
            0, // Volts per cosine of angle
            10.5 / (Math.PI / 2)); // Volts per radians/sec of velocity setpoint
  }

  private enum ControlMode {
    TARGET_ANGLE,
    TARGET_VEL,
    OPEN_LOOP
  }

  private Rotation2d targetAngle = null;
  private Measure<Velocity<Angle>> targetVelocity = RadiansPerSecond.of(0);
  private ControlMode controlMode = ControlMode.TARGET_VEL;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.angle = getAngle();
    inputs.isTooFarDown = getAngle().getDegrees() <= PivotIO.MIN_ANGLE;
    inputs.isTooFarUp = getAngle().getDegrees() >= PivotIO.MAX_ANGLE;

    switch (controlMode) {
      case TARGET_ANGLE:
        targetVelocity =
            RadiansPerSecond.of(
                angleMotionProfile.calculate(
                        Robot.defaultPeriodSecs,
                        new TrapezoidProfile.State(
                            Radians.of(getAngle().getRadians()),
                            DegreesPerSecond.of(leaderEncoder.getVelocity())),
                        new TrapezoidProfile.State(
                            Radians.of(targetAngle.getRadians()), RadiansPerSecond.of(0)))
                    .velocity);
        runMotor(targetVelocity);
        break;
      case TARGET_VEL:
        runMotor(targetVelocity);
        break;
      case OPEN_LOOP:
        leaderMotor.setVoltage(leaderSetpoint);
        followerMotor.setVoltage(followerSetpoint);
    }

    inputs.leaderVelocity = DegreesPerSecond.of(leaderEncoder.getVelocity());
    inputs.followerVelocity = DegreesPerSecond.of(followerEncoder.getVelocity());

    // inputs.appliedVoltage = Volts.of(leaderMotor.getBusVoltage() *
    // leaderMotor.get());
    inputs.leaderAppliedVoltage = Volts.of(leaderSetpoint);
    inputs.followerAppliedVoltage = Volts.of(followerSetpoint);
    inputs.leaderAppliedCurrent = Amps.of(leaderMotor.getOutputCurrent());
    inputs.leaderAppliedCurrent = Amps.of(followerMotor.getOutputCurrent());
  }

  @Override
  public void setTargetAngle(Rotation2d targetAngle) {
    this.targetAngle = targetAngle;
    controlMode = ControlMode.TARGET_ANGLE;
  }

  @Override
  public void setVelocityControl(Measure<Velocity<Angle>> velocity) {
    targetVelocity = velocity;
    controlMode = ControlMode.TARGET_VEL;
  }

  @Override
  public void setRawControl(Measure<Voltage> voltage) {
    controlMode = ControlMode.OPEN_LOOP;
    leaderSetpoint = voltage.in(Volts);
    followerSetpoint = voltage.in(Volts);
  }

  @Override
  public void stop() {
    setVelocityControl(RadiansPerSecond.of(0));
  }

  /** Moving average filter to smooth out the noisy input. */
  private LinearFilter angleFilter = LinearFilter.movingAverage(5);

  /** Gets the angle of the pivot mechanism. */
  private Rotation2d getAngle() {
    double rawAngle = encoder.getAbsolutePosition() - (256.0 / 360.0);
    return Rotation2d.fromRotations(angleFilter.calculate(rawAngle));
  }

  /** Run the motors at the specified pivot velocity. */
  private void runMotor(Measure<Velocity<Angle>> pivotVel) {
    if ((pivotVel.baseUnitMagnitude() < 0 && getAngle().getDegrees() <= PivotIO.MIN_ANGLE)
        || (pivotVel.baseUnitMagnitude() > 0 && getAngle().getDegrees() >= PivotIO.MAX_ANGLE)) {
      pivotVel = RPM.of(0);
    }

    Logger.recordOutput("Shooter/pivot/targetVel", pivotVel.in(RadiansPerSecond));
    leaderSetpoint =
        -(leaderPidController.calculate(-leaderEncoder.getVelocity(), pivotVel.in(DegreesPerSecond))
            + leaderFFController.calculate(getAngle().getRadians(), pivotVel.in(RadiansPerSecond)));
    followerSetpoint =
        -(followerPidController.calculate(
                -followerEncoder.getVelocity(), pivotVel.in(DegreesPerSecond))
            + followerFFController.calculate(
                getAngle().getRadians(), pivotVel.in(RadiansPerSecond)));
    leaderMotor.setVoltage(leaderSetpoint);
    followerMotor.setVoltage(followerSetpoint);
  }
}
