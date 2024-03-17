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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
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
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.CANConstants;
import frc.robot.util.CANConstants.Drivebase;
import frc.robot.util.SwerveUtils;
import java.util.Queue;

/**
 * Module IO implementation for a Talon FX drive motor controller, a Talon FX azimuth motor
 * controller, and CANcoder. Assumes all devices used are Phoenix Pro licensed.
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations.
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
  private Queue<Double> timestampQueue;
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
  private Queue<Double> drivePositionQueue;
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
  private Queue<Double> azimuthPositionQueue;
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
   * @param moduleID The index of the module.
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
  @SuppressWarnings("unused")
  public ModuleIOTalonFX(int moduleID) {
    // PIDF tuning values. NONE OF THESE VALUES SHOULD BE NEGATIVE, IF THEY ARE YA DONE GOOFED
    // SOMEWHERE
    double drive_kS; // Amps of current needed to overcome friction
    // kV should be at/near 0, as kV is usually used to fight against back-EMF and TorqueControlFOC
    // commutation eliminates that issue. However, if the mechanism has a non-negligible amount of
    // viscous friction, ie from grease, then you may need it to counter the friction.
    double drive_kV; // Amps of additional current per m/s of velocity setpoint
    // kA can be tuned independently of all other control parameters. If the actual acceleration is
    // below requested acceleration, bump this up, if it's above requested acceleration, bump this
    // down.
    double drive_kA; // Amps of additional current per m/s^2 of acceleration setpoint
    double drive_kP; // Amps of current per m/s of error
    // kI is typically unnecesary for driving as there's no significant factors that can prevent a
    // PID controller from hitting its target, such as gravity for an arm. Factors like friction and
    // inertia can be accounted for using kS and kA.
    double drive_kI; // Amps of current per m of integrated error
    double drive_kD; // Amps of current per m/s^2 of error derivative
    double azimuth_kS; // Amps of current needed to overcome friction
    double azimuth_kV; // Amps of current per rot/s of velocity setpoint
    double azimuth_kA; // Amps of additional current per rot/s^2 of acceleration setpoint
    double azimuth_kP; // Amps of current per rotation of error
    double azimuth_kI; // Amps of current per rotation of integrated error
    double azimuth_kD; // Amps of current per rotations/s of error derivative
    switch (moduleID) {
      case 0 -> {
        driveTalon = new TalonFX(Drivebase.FRONT_LEFT_DRIVE, CANConstants.CANIVORE_NAME);
        azimuthTalon = new TalonFX(Drivebase.FRONT_LEFT_TURN, CANConstants.CANIVORE_NAME);
        cancoder = new CANcoder(Drivebase.FRONT_LEFT_ENCODER, CANConstants.CANIVORE_NAME);
        absoluteEncoderOffset = Rotation2d.fromDegrees(0); // TODO: Calibrate
        drive_kS = 0;
        drive_kV = 0;
        drive_kA = 2;
        drive_kP = 1;
        drive_kI = 0;
        drive_kD = 0;

        azimuth_kS = 0;
        azimuth_kV = 0;
        azimuth_kA = 2;
        azimuth_kP = 4;
        azimuth_kI = 0;
        azimuth_kD = 0;
      }
      case 1 -> {
        driveTalon = new TalonFX(Drivebase.FRONT_RIGHT_DRIVE, CANConstants.CANIVORE_NAME);
        azimuthTalon = new TalonFX(Drivebase.FRONT_RIGHT_TURN, CANConstants.CANIVORE_NAME);
        cancoder = new CANcoder(Drivebase.FRONT_RIGHT_ENCODER, CANConstants.CANIVORE_NAME);
        absoluteEncoderOffset = Rotation2d.fromDegrees(0); // TODO: Calibrate

        drive_kS = 0;
        drive_kV = 0;
        drive_kA = 2;
        drive_kP = 1;
        drive_kI = 0;
        drive_kD = 0;

        azimuth_kS = 0;
        azimuth_kV = 0;
        azimuth_kA = 2;
        azimuth_kP = 4;
        azimuth_kI = 0;
        azimuth_kD = 0;
      }
      case 2 -> {
        driveTalon = new TalonFX(Drivebase.BACK_LEFT_DRIVE, CANConstants.CANIVORE_NAME);
        azimuthTalon = new TalonFX(Drivebase.BACK_RIGHT_TURN, CANConstants.CANIVORE_NAME);
        cancoder = new CANcoder(Drivebase.BACK_LEFT_ENCODER, CANConstants.CANIVORE_NAME);
        absoluteEncoderOffset = Rotation2d.fromDegrees(0); // TODO: Calibrate

        drive_kS = 0;
        drive_kV = 0;
        drive_kA = 2;
        drive_kP = 1;
        drive_kI = 0;
        drive_kD = 0;

        azimuth_kS = 0;
        azimuth_kV = 0;
        azimuth_kA = 2;
        azimuth_kP = 4;
        azimuth_kI = 0;
        azimuth_kD = 0;
      }
      case 3 -> {
        driveTalon = new TalonFX(Drivebase.BACK_RIGHT_DRIVE, CANConstants.CANIVORE_NAME);
        azimuthTalon = new TalonFX(Drivebase.BACK_RIGHT_TURN, CANConstants.CANIVORE_NAME);
        cancoder = new CANcoder(Drivebase.BACK_RIGHT_ENCODER, CANConstants.CANIVORE_NAME);
        absoluteEncoderOffset = Rotation2d.fromDegrees(0); // TODO: Calibrate

        drive_kS = 0;
        drive_kV = 0;
        drive_kA = 2;
        drive_kP = 1;
        drive_kI = 0;
        drive_kD = 0;

        azimuth_kS = 0;
        azimuth_kV = 0;
        azimuth_kA = 2;
        azimuth_kP = 4;
        azimuth_kI = 0;
        azimuth_kD = 0;
      }
      default -> throw new IndexOutOfBoundsException(
          "Invalid module ID. Expected 0-3, got " + moduleID);
    }

    var driveConfig = new TalonFXConfiguration();
    // The rotations output of the motor encoder will be divided by this value. This shouldn't
    // really be used for unit conversions, as getVelocity has a max value of +-512, but since we're
    // unit converting to meters instead of something like degrees, the max value we're expecting to
    // see when this is applied is +-4.5, so this is probably fine.
    driveConfig.Feedback.SensorToMechanismRatio = // Gear ratio between the motor and the wheel
        Module.DRIVE_GEAR_RATIO / Module.DRIVE_WHEEL_CIRCUMFERENCE.in(Meters);
    driveConfig.Slot0.kS = drive_kS;
    driveConfig.Slot0.kV = drive_kV;
    driveConfig.Slot0.kA = drive_kA;
    driveConfig.Slot0.kP = drive_kP;
    driveConfig.Slot0.kI = drive_kI;
    driveConfig.Slot0.kD = drive_kD;
    driveConfig.MotionMagic.MotionMagicAcceleration = SwerveSubsystem.getMaxLinearAcceleration().in(MetersPerSecondPerSecond); // Max allowed acceleration, in m/s^2
    driveConfig.MotionMagic.MotionMagicJerk = driveConfig.MotionMagic.MotionMagicAcceleration * 5; // Max allowed jerk, in m/s^3
    // Limit the current draw of the motors.
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 100;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = 100;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveTalon.getConfigurator().apply(driveConfig);

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
    azimuthConfig.Slot0.kS = azimuth_kS;
    azimuthConfig.Slot0.kV = azimuth_kV;
    azimuthConfig.Slot0.kA = azimuth_kA;
    azimuthConfig.Slot0.kP = azimuth_kP;
    azimuthConfig.Slot0.kI = azimuth_kI;
    azimuthConfig.Slot0.kD = azimuth_kD;
    azimuthConfig.MotionMagic.MotionMagicAcceleration = 10; // Max allowed acceleration, in rot/s^2
    azimuthConfig.MotionMagic.MotionMagicJerk = 100; // Max allowed jerk, in rot/s^3
    azimuthConfig.MotionMagic.MotionMagicCruiseVelocity = 4.5;
    azimuthConfig.ClosedLoopGeneral.ContinuousWrap = true;
    azimuthConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    azimuthConfig.TorqueCurrent.PeakReverseTorqueCurrent = 40;
    azimuthConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    azimuthTalon.getConfigurator().apply(azimuthConfig);

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    // Set up the StatusSignals for getting values.
    if (Module.ODOMETRY_FREQUENCY > 50) {
      timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
      drivePositionQueue =
          PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
      azimuthPositionQueue =
          PhoenixOdometryThread.getInstance()
              .registerSignal(azimuthTalon, azimuthTalon.getPosition());
    }

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveClosedLoopOutput = driveTalon.getClosedLoopOutput();
    driveAppliedCurrent = driveTalon.getTorqueCurrent();

    azimuthAbsolutePosition = cancoder.getAbsolutePosition();
    azimuthPosition = azimuthTalon.getPosition();
    azimuthVelocity = azimuthTalon.getVelocity();
    azimuthAppliedVolts = azimuthTalon.getMotorVoltage();
    azimuthClosedLoopOutput = azimuthTalon.getClosedLoopOutput();
    azimuthAppliedCurrent = azimuthTalon.getTorqueCurrent();

    // Boost the rate at which position and velocity data for the drive motor and azimuth motor are
    // sent over CAN for async odometry.
    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY > 50 ? Module.ODOMETRY_FREQUENCY : 50,
        drivePosition,
        azimuthPosition);
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
    inputs.driveAppliedCurrent = Amps.of(driveAppliedCurrent.getValueAsDouble());

    inputs.azimuthAbsolutePosition =
        Rotation2d.fromRotations(azimuthAbsolutePosition.getValueAsDouble());
    inputs.azimuthVelocity = RotationsPerSecond.of(azimuthVelocity.getValueAsDouble());
    inputs.azimuthAppliedVoltage = Volts.of(azimuthAppliedVolts.getValueAsDouble());
    inputs.azimuthAppliedOutput = azimuthClosedLoopOutput.getValueAsDouble();
    inputs.azimuthAppliedCurrent = Amps.of(azimuthAppliedCurrent.getValueAsDouble());

    if (timestampQueue != null) {
      inputs.odometryTimestamps = SwerveUtils.queueToDoubleArray(timestampQueue);
      inputs.odometryDrivePositionsMeters = SwerveUtils.queueToDoubleArray(drivePositionQueue);
      inputs.odometryAzimuthPositions = SwerveUtils.queueToRotation2dArray(azimuthPositionQueue);
    }
  }

  /**
   * The control request for accelerating the drive motor up to a specified wheel velocity. Is
   * mutable.
   */
  MotionMagicVelocityTorqueCurrentFOC driveVelocityControlRequest = new MotionMagicVelocityTorqueCurrentFOC(0);

  @Override
  public void setDriveVelocity(Measure<Velocity<Distance>> velocity) {
    driveTalon.setControl(driveVelocityControlRequest);
  }

  /**
   * The control request for accelerating the drive motor using a raw amperage value. Is mutable.
   */
  TorqueCurrentFOC driveCurrentControlRequest = new TorqueCurrentFOC(0);

  @Override
  public void setRawDrive(double rawUnits) {
    driveCurrentControlRequest.Output = rawUnits;
    driveTalon.setControl(driveCurrentControlRequest);
  }

  /**
   * The control request for accelerating the azimuth motor using a raw amperage value. Is mutable.
   */
  TorqueCurrentFOC rawCurrentControlRequest = new TorqueCurrentFOC(0);

  @Override
  public void setRawAzimuth(double rawUnits) {
    rawCurrentControlRequest.Output = rawUnits;
    azimuthTalon.setControl(rawCurrentControlRequest);
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
