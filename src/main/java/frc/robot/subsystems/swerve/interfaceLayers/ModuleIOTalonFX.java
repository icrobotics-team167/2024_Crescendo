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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import frc.robot.subsystems.swerve.Module;
import java.util.Queue;

/**
 * Module IO implementation for a Talon FX drive motor controller, a Talon FX azimuth motor
 * controller, and CANcoder. Assumes all devices used are Phoenix Pro licensed.
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope.
 */
public class ModuleIOTalonFX implements ModuleIO {
  /** The TalonFX motor controller for the drive motor. */
  private final TalonFX driveTalon;
  /** The TalonFX motor controller for the azimuth motor. */
  private final TalonFX azimuthTalon;
  /** The CANcoder for azimuth. */
  private final CANcoder cancoder;

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
   * The current distance that the module has driven so far.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Meters
   *       </ul>
   * </ul>
   */
  private final StatusSignal<Double> drivePosition;
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
   * The current drive velocity of the module.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Meters per second
   *       </ul>
   * </ul>
   */
  private final StatusSignal<Double> driveVelocity;
  /**
   * The voltage applied to the motor by the motor controller.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Volts
   *       </ul>
   * </ul>
   */
  private final StatusSignal<Double> driveAppliedVolts;
  /**
   * The current applied to the motor by the motor controller.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Amps
   *       </ul>
   * </ul>
   */
  private final StatusSignal<Double> driveAppliedCurrent;
  /**
   * The total output applied to the motor by the closed loop control.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Percentage
   *       </ul>
   * </ul>
   */
  private final StatusSignal<Double> driveClosedLoopOutput;
  /**
   * The absolute position of the module azimuth, as measured by an absolute encoder. 0 should mean
   * the module is facing forwards. Wraps [-0.5, 0.5)
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Rotations
   *       </ul>
   * </ul>
   */
  private final StatusSignal<Double> azimuthAbsolutePosition;
  /**
   * The position of the module azimuth, as measured by the azimuth motor's internal encoder. Is
   * periodically synchronized with the absolute position.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Rotations
   *       </ul>
   * </ul>
   */
  private final StatusSignal<Double> azimuthPosition;
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
   * The current velocity of the azimuth.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Rotations per second
   *       </ul>
   * </ul>
   */
  private final StatusSignal<Double> azimuthVelocity;
  /**
   * The voltage applied to the motor by the motor controller.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Volts
   *       </ul>
   * </ul>
   */
  private final StatusSignal<Double> azimuthAppliedVolts;
  /**
   * The current applied to the motor by the motor controller.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Amps
   *       </ul>
   * </ul>
   */
  private final StatusSignal<Double> azimuthAppliedCurrent;
  /**
   * The total output applied to the motor by the closed loop control.
   *
   * <ul>
   *   <li><b>Units:</b>
   *       <ul>
   *         <li>Percentage
   *       </ul>
   * </ul>
   */
  private final StatusSignal<Double> azimuthClosedLoopOutput;
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
   * Constructs a new TalonFX-based swerve module IO interface.
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
  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0: // Front Left module
        driveTalon = new TalonFX(0, "drivebase");
        azimuthTalon = new TalonFX(1, "drivebase");
        cancoder = new CANcoder(2, "drivebase");
        absoluteEncoderOffset = Rotation2d.fromDegrees(0); // TODO: Calibrate
        break;
      case 1: // Front Right modules
        driveTalon = new TalonFX(3, "drivebase");
        azimuthTalon = new TalonFX(4, "drivebase");
        cancoder = new CANcoder(5, "drivebase");
        absoluteEncoderOffset = Rotation2d.fromDegrees(0); // TODO: Calibrate
        break;
      case 2: // Back Left modules
        driveTalon = new TalonFX(6, "drivebase");
        azimuthTalon = new TalonFX(7, "drivebase");
        cancoder = new CANcoder(8, "drivebase");
        absoluteEncoderOffset = Rotation2d.fromDegrees(0); // TODO: Calibrate
        break;
      case 3: // Back Right modules
        driveTalon = new TalonFX(9, "drivebase");
        azimuthTalon = new TalonFX(10, "drivebase");
        cancoder = new CANcoder(11, "drivebase");
        absoluteEncoderOffset = Rotation2d.fromDegrees(0); // TODO: Calibrate
        break;
      default: // If somehow a 5th module is constructed, error
        throw new RuntimeException("Invalid module index");
    }

    var driveConfig = new TalonFXConfiguration();
    // The rotations output of the motor encoder will be divided by this value. This shouldn't
    // really be used for unit conversions, as getVelocity has a max value of +-512, but since we're
    // unit converting to meters instead of something like degrees, the max value we're expecting to
    // see when this is applied is +-4.5, so this is probably fine.
    driveConfig.Feedback.SensorToMechanismRatio = // Gear ratio between the motor and the wheel
        Module.DRIVE_GEAR_RATIO / Module.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters);
    // PIDF tuning values. NONE OF THESE VALUES SHOULD BE NEGATIVE, IF THEY ARE YA DONE GOOFED
    // SOMEWHERE
    driveConfig.Slot0.kP = 0.05; // % output per m/s of error
    // kI is typically unnecesary for driving as there's no significant factors that can prevent a
    // PID controller from hitting its target, such as gravity for an arm. Factors like friction and
    // inertia can be accounted for using kS and kA.
    driveConfig.Slot0.kI = 0; // % output per m of integrated error
    driveConfig.Slot0.kD = 0; // % output per m/s^2 of error derivative
    driveConfig.Slot0.kS = 0; // Amps of additional current needed to overcome friction
    // kV should be at/near 0, as kV is usually used to fight against back-EMF and TorqueControlFOC
    // commutation eliminates that issue. However, if the mechanism has a non-negligible amount of
    // viscous friction, ie from grease, then you may need it to counter the friction. See
    // https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#choosing-output-type
    driveConfig.Slot0.kV = 0; // Amps of additional current per m/s of velocity setpoint
    // kA can be tuned independently of all over control parameters. If the actual acceleration is
    // below requested acceleration, bump this up, if it's above requested acceleration, bump this
    // down.
    driveConfig.Slot0.kA = 0; // Amps of additional current per m/s^2 of acceleration setpoint
    // MotionMagicAcceleration should be close to the maximum acceleration you can handle given your
    // robot's mass and moment of inertia. Choreo has a tool to approximate this.
    driveConfig.MotionMagic.MotionMagicAcceleration = 14; // Max allowed acceleration, in m/s^2
    // MotionMagicJerk should be ~10-20x the acceleration value, meaning roughly 0.05-0.1 seconds to
    // max acceleration. Is mainly used to smooth out motion profiles.
    driveConfig.MotionMagic.MotionMagicJerk = 140; // Max allowed jerk, in m/s^3
    // Limit the current draw of the motors.
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 100;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = 100;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    // See drive config for comments on these, similar concepts apply for azimuth.
    var azimuthConfig = new TalonFXConfiguration();
    azimuthConfig.Feedback.SensorToMechanismRatio = 1;
    azimuthConfig.Feedback.RotorToSensorRatio = Module.AZIMUTH_GEAR_RATIO;
    azimuthConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    azimuthConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    azimuthConfig.MotorOutput.Inverted =
        Module.AZIMUTH_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    azimuthConfig.Slot0.kP = 1; // % output per rotation of error
    azimuthConfig.Slot0.kI = 0; // % output per rotation of integrated error
    azimuthConfig.Slot0.kD = 0; // % output per rotations/s of error derivative
    azimuthConfig.Slot0.kS = 0; // Amps of additional current needed to overcome friction
    azimuthConfig.Slot0.kV = 0; // Amps of additional current per rot/s of velocity setpoint
    azimuthConfig.Slot0.kA = 0; // Amps of additional current per rot/s^2 of acceleration setpoint
    azimuthConfig.MotionMagic.MotionMagicAcceleration = 5; // Max allowed acceleration, in rot/s^2
    azimuthConfig.MotionMagic.MotionMagicJerk = 50; // Max allowed jerk, in rot/s^3
    azimuthConfig.ClosedLoopGeneral.ContinuousWrap = true;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = 40;
    azimuthTalon.getConfigurator().apply(azimuthConfig);
    setAzimuthBrakeMode(true);

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    // Set up the StatusSignals for getting values.
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveClosedLoopOutput = driveTalon.getClosedLoopOutput();
    driveAppliedCurrent = driveTalon.getTorqueCurrent();

    azimuthAbsolutePosition = cancoder.getAbsolutePosition();
    azimuthPosition = azimuthTalon.getPosition();
    azimuthPositionQueue =
        PhoenixOdometryThread.getInstance()
            .registerSignal(azimuthTalon, azimuthTalon.getPosition());
    azimuthVelocity = azimuthTalon.getVelocity();
    azimuthAppliedVolts = azimuthTalon.getMotorVoltage();
    azimuthClosedLoopOutput = azimuthTalon.getClosedLoopOutput();
    azimuthAppliedCurrent = azimuthTalon.getTorqueCurrent();

    // Boost the rate at which position and velocity data for the drive motor and azimuth motor are
    // sent over CAN for async odometry.
    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, azimuthPosition);
    // Lower the rate at which everything else we need is send over CAN to reduce CAN bus usage.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveClosedLoopOutput,
        driveAppliedCurrent,
        azimuthAbsolutePosition,
        azimuthVelocity,
        azimuthAppliedVolts,
        azimuthClosedLoopOutput,
        azimuthAppliedCurrent);
    // Completely disable sending of any data we don't need to further reduce CAN bus usage.
    driveTalon.optimizeBusUtilization();
    azimuthTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveClosedLoopOutput,
        driveAppliedCurrent,
        azimuthAbsolutePosition,
        azimuthPosition,
        azimuthVelocity,
        azimuthAppliedVolts,
        azimuthClosedLoopOutput,
        azimuthAppliedCurrent);

    inputs.drivePosition = Meters.of(drivePosition.getValueAsDouble());
    inputs.driveVelocity = MetersPerSecond.of(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVoltage = Volts.of(driveAppliedVolts.getValueAsDouble());
    inputs.driveAppliedOutput = driveClosedLoopOutput.getValueAsDouble();
    inputs.driveAppliedCurrentAmps = Amps.of(driveAppliedCurrent.getValueAsDouble());

    inputs.azimuthAbsolutePosition =
        Rotation2d.fromRotations(azimuthAbsolutePosition.getValueAsDouble());
    inputs.azimuthVelocity = RotationsPerSecond.of(azimuthVelocity.getValueAsDouble());
    inputs.azimuthAppliedVoltage = Volts.of(azimuthAppliedVolts.getValueAsDouble());
    inputs.azimuthAppliedOutput = azimuthClosedLoopOutput.getValueAsDouble();
    inputs.azimuthAppliedCurrentAmps = Amps.of(azimuthAppliedCurrent.getValueAsDouble());

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

  /**
   * The control request for accelerating the drive motor up to a specified wheel velocity. Is
   * mutable.
   */
  MotionMagicVelocityTorqueCurrentFOC driveControlRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  @Override
  public void setDriveVelocity(Measure<Velocity<Distance>> velocity) {
    driveTalon.setControl(driveControlRequest);
  }

  /**
   * The control request for accelerating the drive motor using a raw amperage value. Is mutable.
   */
  TorqueCurrentFOC rawCurrentControlRequest = new TorqueCurrentFOC(0);

  @Override
  public void setRawDrive(double rawUnits) {
    rawCurrentControlRequest.Output = rawUnits;
    driveTalon.setControl(rawCurrentControlRequest);
  }

  /** The control request for moving the azimuth motor to a specified position. Is mutable. */
  MotionMagicTorqueCurrentFOC azimuthControlRequest = new MotionMagicTorqueCurrentFOC(0);

  @Override
  public void setAzimuthPosition(Rotation2d position) {
    azimuthControlRequest.Position = position.getRotations();
    azimuthTalon.setControl(azimuthControlRequest);
  }

  @Override
  public void stop() {
    driveTalon.stopMotor();
    driveTalon.stopMotor();
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setAzimuthBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        Module.AZIMUTH_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    azimuthTalon.getConfigurator().apply(config);
  }

  @Override
  public void configureDriveSysID() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, driveVelocity, drivePosition, driveAppliedCurrent);
  }

  @Override
  public void configureAzimuthSysID() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, azimuthVelocity, azimuthPosition, azimuthAppliedCurrent);
  }
}
