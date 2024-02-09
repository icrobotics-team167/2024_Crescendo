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

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Module;
import frc.robot.util.SparkUtils;
import java.util.Queue;
import java.util.Set;

/**
 * Module IO implementation for a Spark Max drive motor controller, Spark Max azimuth motor
 * controller (NEO or NEO 550), and an analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope.
 */
public class ModuleIOSparkMax implements ModuleIO {
  /** The Spark Max motor controller for the drive motor. */
  private final CANSparkMax driveSparkMax;
  /** The Spark Max motor controller for the azimuth motor. */
  private final CANSparkMax azimuthSparkMax;

  /** The internal encoder of the drive motor. */
  private final RelativeEncoder driveEncoder;
  /** The internal encoder of the azimuth motor. */
  private final RelativeEncoder azimuthRelativeEncoder;
  /** The PID controller for the drive motor. */
  private final SparkPIDController drivePIDController;
  /** The FF constants of the drive motor. */
  private final SimpleMotorFeedforward driveFF;
  /** The PID controller for the azimuth motor. */
  private final SparkPIDController azimuthPIDController;
  /** The absolute encoder for azimuth. */
  private final AnalogInput azimuthAbsoluteEncoder;
  /**
   * A {@link Queue} holding all the timestamps that the async odometry thread captures.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Seconds
   *       </ul>
   * </ul>
   */
  private final Queue<Double> timestampQueue;
  /**
   * A {@link Queue} holding all the drive positions that the async odometry thread captures.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Meters
   *       </ul>
   * </ul>
   */
  private final Queue<Double> drivePositionQueue;
  /**
   * A {@link Queue} holding all the azimuth positions that the async odometry thread captures.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Rotations
   *       </ul>
   * </ul>
   */
  private final Queue<Double> azimuthPositionQueue;
  /**
   * Due to the nature of mounting magnets for absolute encoders, it is practically impossible to
   * line up magnetic north with forwards on the module. This value is subtracted from the raw
   * detected position, such that 0 is actually forwards on the azimuth.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Rotations
   *       </ul>
   * </ul>
   */
  private final Rotation2d absoluteEncoderOffset;

  /**
   * Constructs a new Spark Max-based swerve module IO interface.
   *
   * @param index The index of the module.
   *     <ul>
   *       <li><b>Position of module from index</b>
   *           <ul>
   *             <li>0: Front Left
   *             <li>1: Front Right
   *             <li>2: Back Left
   *             <li>3: Back Right
   *           </ul>
   *     </ul>
   */
  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0: // Front Left
        driveSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        azimuthSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new AnalogInput(0);
        absoluteEncoderOffset = new Rotation2d(0.0); // TODO: Calibrate
        break;
      case 1: // Front Right
        driveSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        azimuthSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new AnalogInput(1);
        absoluteEncoderOffset = new Rotation2d(0.0); // TODO: Calibrate
        break;
      case 2: // Back Left
        driveSparkMax = new CANSparkMax(5, MotorType.kBrushless);
        azimuthSparkMax = new CANSparkMax(6, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new AnalogInput(2);
        absoluteEncoderOffset = new Rotation2d(0.0); // TODO: Calibrate
        break;
      case 3: // Back Right
        driveSparkMax = new CANSparkMax(7, MotorType.kBrushless);
        azimuthSparkMax = new CANSparkMax(8, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new AnalogInput(3);
        absoluteEncoderOffset = new Rotation2d(0.0); // TODO: Calibrate
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    // Configure motors
    driveSparkMax.restoreFactoryDefaults();
    azimuthSparkMax.restoreFactoryDefaults();

    // Set up a timeout for applying settings.
    driveSparkMax.setCANTimeout(250);
    azimuthSparkMax.setCANTimeout(250);

    SparkUtils.configureSettings(false, IdleMode.kBrake, Amps.of(100), driveSparkMax);
    SparkUtils.configureSettings(
        Module.AZIMUTH_MOTOR_INVERTED, IdleMode.kBrake, Amps.of(40), azimuthSparkMax);

    // Initialize encoders
    driveEncoder = driveSparkMax.getEncoder();
    azimuthRelativeEncoder = azimuthSparkMax.getEncoder();

    // The motor output in rotations is multiplied by this factor.
    driveEncoder.setPositionConversionFactor(
        Module.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters) / Module.DRIVE_GEAR_RATIO);
    driveEncoder.setVelocityConversionFactor(
        (Module.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters) / Module.DRIVE_GEAR_RATIO) / 60);
    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    azimuthRelativeEncoder.setPositionConversionFactor(1.0 / Module.AZIMUTH_GEAR_RATIO);
    azimuthRelativeEncoder.setVelocityConversionFactor((1.0 / Module.AZIMUTH_GEAR_RATIO) / 60);
    azimuthRelativeEncoder.setPosition(getAzimuthAbsolutePosition().getRotations());
    azimuthRelativeEncoder.setMeasurementPeriod(10);
    azimuthRelativeEncoder.setAverageDepth(2);

    // PIDF tuning values. NONE OF THESE VALUES SHOULD BE NEGATIVE, IF THEY ARE YA DONE GOOFED
    // SOMEWHERE
    drivePIDController = driveSparkMax.getPIDController();
    drivePIDController.setP(0.05); // % Output per m/s of error
    // kI is typically unnecesary for driving as there's no significant factors that can prevent a
    // PID controller from hitting its target, such as gravity for an arm. Factors like friction and
    // inertia can be accounted for using kS and kV.
    drivePIDController.setI(0); // % Output per m of integrated error
    drivePIDController.setD(0); // % Output per m/s^2 of error derivative
    driveFF =
        new SimpleMotorFeedforward(
            0, // Volts of additional voltage needed to overcome friction
            0); // Volts of additional voltage per m/s of velocity setpoint

    azimuthPIDController = azimuthSparkMax.getPIDController();
    azimuthPIDController.setP(1); // % Output per rotation of error
    azimuthPIDController.setI(0); // % Output per rotation of integrated error
    azimuthPIDController.setD(0); // % Output per rotations/s of error derivative
    azimuthPIDController.setFF(0); // Volts of additional voltage per rot/s of velocity
    azimuthPIDController.setPositionPIDWrappingEnabled(true);
    azimuthPIDController.setPositionPIDWrappingMaxInput(0.5);
    azimuthPIDController.setPositionPIDWrappingMinInput(-0.5);

    driveSparkMax.burnFlash();
    azimuthSparkMax.burnFlash();
    driveSparkMax.setCANTimeout(0);
    azimuthSparkMax.setCANTimeout(0);

    // Configure CAN frame usage, and disable any unused CAN frames.
    SparkUtils.configureFrameStrategy(
        driveSparkMax,
        Set.of(SparkUtils.Data.VELOCITY, SparkUtils.Data.VOLTAGE, SparkUtils.Data.CURRENT),
        Set.of(SparkUtils.Sensor.INTEGRATED),
        false);
    SparkUtils.configureFrameStrategy(
        azimuthSparkMax,
        Set.of(SparkUtils.Data.VELOCITY, SparkUtils.Data.VOLTAGE, SparkUtils.Data.CURRENT),
        Set.of(SparkUtils.Sensor.INTEGRATED),
        false);

    // Set up high frequency odometry
    driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    azimuthSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    azimuthPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(azimuthRelativeEncoder::getPosition);

    driveSparkMax.burnFlash();
    azimuthSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveAppliedOutput = driveSparkMax.getAppliedOutput();
    // Why does REV not have a stator voltage getter
    inputs.driveAppliedVoltage =
        Volts.of(inputs.driveAppliedOutput * driveSparkMax.getBusVoltage());
    inputs.driveAppliedCurrentAmps = Amps.of(driveSparkMax.getOutputCurrent());
    inputs.drivePosition = Meters.of(driveEncoder.getPosition());
    inputs.driveVelocity = MetersPerSecond.of(driveEncoder.getVelocity());

    inputs.azimuthAbsolutePosition = getAzimuthAbsolutePosition();
    azimuthRelativeEncoder.setPosition(inputs.azimuthAbsolutePosition.getRotations());
    inputs.azimuthVelocity = RotationsPerSecond.of(azimuthRelativeEncoder.getVelocity());
    inputs.azimuthAppliedOutput = driveSparkMax.getAppliedOutput();
    inputs.azimuthAppliedVoltage =
        Volts.of(inputs.azimuthAppliedOutput * driveSparkMax.getBusVoltage());
    inputs.azimuthAppliedCurrentAmps = Amps.of(azimuthSparkMax.getOutputCurrent());

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryAzimuthPositions =
        azimuthPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    azimuthPositionQueue.clear();
  }

  @Override
  public void setDriveVelocity(Measure<Velocity<Distance>> velocity) {
    drivePIDController.setReference(
        velocity.in(MetersPerSecond),
        ControlType.kVelocity,
        0,
        driveFF.calculate(velocity.in(MetersPerSecond)));
  }

  @Override
  public void setRawDrive(double voltage) {
    driveSparkMax.setVoltage(voltage);
  }

  @Override
  public void setAzimuthPosition(Rotation2d position) {
    azimuthPIDController.setReference(position.getRotations(), ControlType.kPosition);
  }

  @Override
  public void stop() {
    driveSparkMax.stopMotor();
    azimuthSparkMax.stopMotor();
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setAzimuthBrakeMode(boolean enable) {
    azimuthSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  private Rotation2d getAzimuthAbsolutePosition() {
    return Rotation2d.fromRotations(
            azimuthAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V())
        .minus(absoluteEncoderOffset);
  }
}
