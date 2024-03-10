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

package frc.robot.subsystems.swerve.interfaceLayers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.motorUtils.SparkUtils.Data.*;
import static frc.robot.util.motorUtils.SparkUtils.Sensor.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.Module;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.CANConstants;
import frc.robot.util.motorUtils.SparkUtils;
import java.util.Set;

public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveMotor;
  private final CANSparkMax azimuthMotor;
  private final CANcoder azimuthCANcoder;

  private final RelativeEncoder driveRelativeEncoder;

  private final PIDController drivePIDs;
  private final SimpleMotorFeedforward driveFF;
  private double driveOutput = 0;

  private final PIDController azimuthPIDs;
  private double azimuthOutput = 0;

  private final StatusSignal<Double> azimuthAbsolutePosition;
  private final StatusSignal<Double> azimuthVelocity;

  private final SlewRateLimiter driveRateLimiter;

  public ModuleIOSparkMax(int moduleID) {
    double drive_kS; // Volts to overcome static friction
    double drive_kV; // Volts per meters/second of setpoint
    double drive_kP; // Volts per meters/second of error
    double drive_kD; // Volts per meters/second^2 or error derivative
    double azimuth_kP; // Volts per rotation of error
    double azimuth_kD; // Volts per rotation/second of error derivative
    double azimuthOffset;
    switch (moduleID) {
      case 0:
        driveMotor = new CANSparkMax(CANConstants.Drivebase.FRONT_LEFT_DRIVE, MotorType.kBrushless);
        azimuthMotor =
            new CANSparkMax(CANConstants.Drivebase.FRONT_LEFT_TURN, MotorType.kBrushless);
        azimuthCANcoder =
            new CANcoder(CANConstants.Drivebase.FRONT_LEFT_ENCODER, CANConstants.CANIVORE_NAME);

        drive_kS = 0;
        drive_kV =
            12.984007025684262497741538719766
                / SwerveSubsystem.MAX_LINEAR_SPEED.in(MetersPerSecond);
        drive_kP = 2;
        drive_kD = 0.00;

        azimuth_kP = 36;
        azimuth_kD = 1;
        azimuthOffset = 0.219482421875;
        break;
      case 1:
        driveMotor =
            new CANSparkMax(CANConstants.Drivebase.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
        azimuthMotor =
            new CANSparkMax(CANConstants.Drivebase.FRONT_RIGHT_TURN, MotorType.kBrushless);
        azimuthCANcoder =
            new CANcoder(CANConstants.Drivebase.FRONT_RIGHT_ENCODER, CANConstants.CANIVORE_NAME);

        drive_kS = 0;
        drive_kV =
            12.229149534069128388467724405323
                / SwerveSubsystem.MAX_LINEAR_SPEED.in(MetersPerSecond);
        drive_kP = 2;
        drive_kD = 0.00;

        azimuth_kP = 36;
        azimuth_kD = 1;
        azimuthOffset = 0.37841796875;
        break;
      case 2:
        driveMotor = new CANSparkMax(CANConstants.Drivebase.BACK_LEFT_DRIVE, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(CANConstants.Drivebase.BACK_LEFT_TURN, MotorType.kBrushless);
        azimuthCANcoder =
            new CANcoder(CANConstants.Drivebase.BACK_LEFT_ENCODER, CANConstants.CANIVORE_NAME);

        drive_kS = 0;
        drive_kV =
            13.238589093647430486949248563446
                / SwerveSubsystem.MAX_LINEAR_SPEED.in(MetersPerSecond);
        drive_kP = 2;
        drive_kD = 0.00;

        azimuth_kP = 36;
        azimuth_kD = 1;
        azimuthOffset = 0.315673828125;
        break;
      case 3:
        driveMotor = new CANSparkMax(CANConstants.Drivebase.BACK_RIGHT_DRIVE, MotorType.kBrushless);
        azimuthMotor =
            new CANSparkMax(CANConstants.Drivebase.BACK_RIGHT_TURN, MotorType.kBrushless);
        azimuthCANcoder =
            new CANcoder(CANConstants.Drivebase.BACK_RIGHT_ENCODER, CANConstants.CANIVORE_NAME);

        drive_kS = 0;
        drive_kV =
            12.229147971409063634201277485683
                / SwerveSubsystem.MAX_LINEAR_SPEED.in(MetersPerSecond);
        drive_kP = 2;
        drive_kD = 0.00;

        azimuth_kP = 36;
        azimuth_kD = 1;
        azimuthOffset = -0.01953125;
        break;
      default:
        throw new IndexOutOfBoundsException("Invalid module ID. Expected 0-3, got " + moduleID);
    }

    SparkUtils.configureSpark(() -> driveMotor.restoreFactoryDefaults());
    SparkUtils.configureSpark(() -> azimuthMotor.restoreFactoryDefaults());
    Timer.delay(0.1);

    SparkUtils.configureSpark(() -> driveMotor.clearFaults());
    SparkUtils.configureSpark(() -> azimuthMotor.clearFaults());
    azimuthCANcoder.clearStickyFaults();
    Timer.delay(0.1);

    SparkUtils.configureSpark(() -> driveMotor.setCANTimeout(250));
    SparkUtils.configureSpark(() -> azimuthMotor.setCANTimeout(250));

    SparkUtils.configureSpark(() -> driveMotor.setIdleMode(IdleMode.kBrake));
    SparkUtils.configureSpark(() -> driveMotor.setSmartCurrentLimit(60));
    SparkUtils.configureSpark(() -> driveMotor.setSecondaryCurrentLimit(80));

    driveRelativeEncoder = driveMotor.getEncoder();
    // Default measurement values are a burning pile of dogshit because REV, why would they
    // willingly set the default measurements to have 112 ms of measurement latency
    // This sets the measurement latency to 24 ms, much more tolerable
    // Formula for measurement delay:
    // T = Moving average filter sample count
    // P = Measurement period
    // Measurement latency = (T-1)/2 * P
    SparkUtils.configureSpark(() -> driveRelativeEncoder.setAverageDepth(4));
    SparkUtils.configureSpark(() -> driveRelativeEncoder.setMeasurementPeriod(16));
    // Convert from rotations/RPM of motor shaft to meters/meters per second of wheel
    SparkUtils.configureSpark(
        () ->
            driveRelativeEncoder.setPositionConversionFactor(
                Module.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters) / Module.DRIVE_GEAR_RATIO));
    SparkUtils.configureSpark(
        () ->
            driveRelativeEncoder.setVelocityConversionFactor(
                (Module.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters) / Module.DRIVE_GEAR_RATIO) / 60));

    drivePIDs = new PIDController(drive_kP, 0, drive_kD);
    driveFF = new SimpleMotorFeedforward(drive_kS, drive_kV);

    driveRateLimiter =
        new SlewRateLimiter(SwerveSubsystem.MAX_LINEAR_ACCELERATION.in(MetersPerSecondPerSecond));

    SparkUtils.configureSpark(() -> azimuthMotor.setIdleMode(IdleMode.kBrake));
    SparkUtils.configureSpark(() -> azimuthMotor.setSmartCurrentLimit(40));
    SparkUtils.configureSpark(() -> azimuthMotor.setSecondaryCurrentLimit(60));
    azimuthMotor.setInverted(Module.AZIMUTH_MOTOR_INVERTED);

    azimuthPIDs = new PIDController(azimuth_kP, 0, azimuth_kD);
    azimuthPIDs.enableContinuousInput(-0.5, 0.5);

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = azimuthOffset;
    azimuthCANcoder.getConfigurator().apply(cancoderConfig);
    azimuthAbsolutePosition = azimuthCANcoder.getAbsolutePosition();
    azimuthAbsolutePosition.setUpdateFrequency(50);
    azimuthVelocity = azimuthCANcoder.getVelocity();
    azimuthVelocity.setUpdateFrequency(50);

    azimuthCANcoder.optimizeBusUtilization();
    SparkUtils.configureFrameStrategy(
        driveMotor, Set.of(POSITION, VELOCITY, OUTPUT, INPUT, CURRENT), Set.of(INTEGRATED), false);
    SparkUtils.configureFrameStrategy(
        azimuthMotor,
        Set.of(POSITION, VELOCITY, OUTPUT, INPUT, CURRENT),
        Set.of(INTEGRATED),
        false);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePosition = Meters.of(driveRelativeEncoder.getPosition());
    inputs.driveVelocity = MetersPerSecond.of(driveRelativeEncoder.getVelocity());
    inputs.driveAppliedOutput = driveOutput;
    inputs.driveAppliedVoltage = Volts.of(driveOutput);
    inputs.driveAppliedCurrent = Amps.of(driveMotor.getOutputCurrent());

    BaseStatusSignal.refreshAll(azimuthAbsolutePosition, azimuthVelocity);
    inputs.azimuthAbsolutePosition =
        Rotation2d.fromRotations(azimuthAbsolutePosition.getValueAsDouble());
    inputs.azimuthVelocity = RotationsPerSecond.of(azimuthVelocity.getValueAsDouble());
    inputs.azimuthAppliedOutput = azimuthOutput;
    inputs.azimuthAppliedVoltage = Volts.of(azimuthOutput);
    inputs.azimuthAppliedCurrent = Amps.of(azimuthMotor.getOutputCurrent());

    // We don't do HF odometry on smaxes. Because REV.
  }

  @Override
  public void setRawDrive(double volts) {
    driveOutput = volts;
    driveMotor.setVoltage(volts);
  }

  @Override
  public void setDriveVelocity(Measure<Velocity<Distance>> velocity) {
    velocity = MetersPerSecond.of(driveRateLimiter.calculate(velocity.in(MetersPerSecond)));
    driveOutput =
        drivePIDs.calculate(driveRelativeEncoder.getVelocity(), velocity.in(MetersPerSecond))
            + driveFF.calculate(velocity.in(MetersPerSecond));
    // driveOutput = 0;
    driveMotor.setVoltage(driveOutput);
  }

  @Override
  public void setAzimuthPosition(Rotation2d position) {
    azimuthAbsolutePosition.refresh();
    azimuthOutput =
        azimuthPIDs.calculate(azimuthAbsolutePosition.getValueAsDouble(), position.getRotations());
    azimuthMotor.setVoltage(azimuthOutput);
  }

  @Override
  public void stop() {
    driveRateLimiter.reset(0);
    driveMotor.stopMotor();
    azimuthMotor.stopMotor();
  }
}
