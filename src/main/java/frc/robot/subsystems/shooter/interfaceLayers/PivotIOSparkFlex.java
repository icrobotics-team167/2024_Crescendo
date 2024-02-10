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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Robot;
import frc.robot.util.SparkUtils;
import java.util.Set;

public class PivotIOSparkFlex implements PivotIO {
  private DutyCycleEncoder encoder;
  private final CANSparkFlex motor;
  private final RelativeEncoder motorEncoder;
  private final TrapezoidProfile motionProfiler;
  private final PIDController pidController;
  private final ArmFeedforward ffController;

  public PivotIOSparkFlex() {

    encoder = new DutyCycleEncoder(0);

    motionProfiler = new TrapezoidProfile(new Constraints(5, 10));
    pidController = new PIDController(0, 0, 0);
    ffController = new ArmFeedforward(0, 0, 0);
    motor = new CANSparkFlex(20, MotorType.kBrushless);
    motorEncoder = motor.getEncoder();
    SparkUtils.configureSettings(false, IdleMode.kBrake, Amps.of(60), motor);
    SparkUtils.configureFrameStrategy(
        motor,
        Set.of(
            SparkUtils.Data.POSITION,
            SparkUtils.Data.VELOCITY,
            SparkUtils.Data.VOLTAGE,
            SparkUtils.Data.CURRENT),
        Set.of(SparkUtils.Sensor.INTEGRATED),
        true);
    CANSparkFlex follower = new CANSparkFlex(19, MotorType.kBrushless);
    follower.setIdleMode(IdleMode.kBrake);
    follower.follow(motor, true);
    follower.setSmartCurrentLimit(60);
    SparkUtils.configureFollowerFrameStrategy(follower);
  }

  private Rotation2d targetAngle = null;
  private double controlVoltage = 0;
  private boolean pidControl = false;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.angle = getAngle();
    inputs.isTooFarDown = getAngle().getDegrees() <= 38; // TODO: Tune
    inputs.isTooFarUp = getAngle().getDegrees() >= 90; // TODO: Tune

    double outputVoltage;
    if (pidControl) {
      // Possibly wack motion profile code...
      // TODO: Check if this works
      State motionProfileState =
          motionProfiler.calculate(
              Robot.defaultPeriodSecs,
              new State(getAngle().getRotations(), motorEncoder.getVelocity() / 60),
              new State(targetAngle.getRotations(), 0));
      outputVoltage =
          pidController.calculate(getAngle().getRotations(), motionProfileState.position)
              + ffController.calculate(getAngle().getRadians(), motionProfileState.velocity);
    } else {
      outputVoltage = controlVoltage;
    }
    if ((outputVoltage > 0 && inputs.isTooFarUp) || (outputVoltage < 0 && inputs.isTooFarDown)) {
      outputVoltage = 0;
    }
    motor.setVoltage(outputVoltage);
    // motor.set(outputVoltage / 12);

    inputs.velocity = RPM.of(motorEncoder.getVelocity());

    inputs.appliedOutput = motor.get();
    inputs.appliedVoltage = Volts.of(motor.getBusVoltage() * motor.get());
    inputs.appliedCurrent = Amps.of(motor.getOutputCurrent());
  }

  @Override
  public void setTargetAngle(Rotation2d targetAngle) {
    this.targetAngle = targetAngle;
    controlVoltage = 0;
    pidControl = true;
  }

  @Override
  public void setPivotControl(Measure<Voltage> rawVolts) {
    this.targetAngle = null;
    controlVoltage = rawVolts.in(Volts);
    pidControl = false;
  }

  @Override
  public void stop() {
    setPivotControl(Volts.of(0));
  }

  private LinearFilter angleFilter = LinearFilter.movingAverage(5);
  private Rotation2d getAngle() {
    double rawAngle = encoder.getAbsolutePosition() - 195.0 / 360.0;
    return Rotation2d.fromRotations(angleFilter.calculate(rawAngle));
  }
}
