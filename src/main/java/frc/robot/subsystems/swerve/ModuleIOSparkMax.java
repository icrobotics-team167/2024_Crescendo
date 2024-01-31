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

package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePosition"
 */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final SparkPIDController drivePIDController;
  private final SimpleMotorFeedforward driveFF;
  private final SparkPIDController turnPIDController;
  private final AnalogInput turnAbsoluteEncoder;
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        driveSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(0);
        absoluteEncoderOffset = new Rotation2d(0.0); // TODO: Calibrate
        break;
      case 1:
        driveSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(1);
        absoluteEncoderOffset = new Rotation2d(0.0); // TODO: Calibrate
        break;
      case 2:
        driveSparkMax = new CANSparkMax(5, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(6, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(2);
        absoluteEncoderOffset = new Rotation2d(0.0); // TODO: Calibrate
        break;
      case 3:
        driveSparkMax = new CANSparkMax(7, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(8, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(3);
        absoluteEncoderOffset = new Rotation2d(0.0); // TODO: Calibrate
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit(40);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPositionConversionFactor(
        Module.DRIVE_WHEEL_CIRCUMFERENCE / Module.DRIVE_GEAR_RATIO);
    driveEncoder.setVelocityConversionFactor(
        (Module.DRIVE_WHEEL_CIRCUMFERENCE / Module.DRIVE_GEAR_RATIO) / 60);
    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPositionConversionFactor(1.0 / Module.TURN_GEAR_RATIO);
    turnRelativeEncoder.setVelocityConversionFactor((1.0 / Module.TURN_GEAR_RATIO) / 60);
    turnRelativeEncoder.setPosition(getTurnAbsolutePosition().getRotations());
    turnRelativeEncoder.setInverted(isTurnMotorInverted);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    drivePIDController = driveSparkMax.getPIDController();
    drivePIDController.setP(0.05); // % Output per m/s of error
    drivePIDController.setI(0); // % Output per m of integrated error
    drivePIDController.setD(0); // % Output per m/s^2 of error derivative
    driveFF =
        new SimpleMotorFeedforward(
            0, // Volts of additional voltage needed to overcome friction
            0); // Volts of additional voltage per m/s of velocity setpoint

    turnPIDController = turnSparkMax.getPIDController();
    turnPIDController.setP(1); // % Output per rotation of error
    turnPIDController.setI(0); // % Output per rotation of integrated error
    turnPIDController.setD(0); // % Output per rotations/s of error derivative
    turnPIDController.setFF(0); // Volts of additional voltage per rot/s of velocity
    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMaxInput(0.5);
    turnPIDController.setPositionPIDWrappingMinInput(-0.5);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(turnRelativeEncoder::getPosition);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveAppliedDutyCycle = driveSparkMax.getAppliedOutput();
    // Why does REV not have a stator voltage getter
    inputs.driveAppliedVolts = inputs.driveAppliedDutyCycle * driveSparkMax.getBusVoltage();
    inputs.driveAppliedCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();

    inputs.turnAbsolutePosition = getTurnAbsolutePosition();
    turnRelativeEncoder.setPosition(inputs.turnAbsolutePosition.getRotations());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnRelativeEncoder.getVelocity());
    inputs.turnAppliedOutput = driveSparkMax.getAppliedOutput();
    inputs.driveAppliedVolts = inputs.turnAppliedOutput * driveSparkMax.getBusVoltage();
    inputs.turnAppliedCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVelocity(double velocity) {
    drivePIDController.setReference(
        velocity, ControlType.kVelocity, 0, driveFF.calculate(velocity));
  }

  @Override
  public void setRawDrive(double voltage) {
    driveSparkMax.setVoltage(voltage);
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    turnPIDController.setReference(position.getRotations(), ControlType.kPosition);
  }

  @Override
  public void stop() {
    driveSparkMax.stopMotor();
    turnSparkMax.stopMotor();
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  private Rotation2d getTurnAbsolutePosition() {
    return Rotation2d.fromRotations(
            turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V())
        .minus(absoluteEncoderOffset);
  }
}
