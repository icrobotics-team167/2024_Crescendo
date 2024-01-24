// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and
 * CANcoder
 *
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware
 * configurations (e.g. If using an analog encoder, copy from
 * "ModuleIOSparkMax")
 *
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward
 * motion on the drive motor will propel the robot forward) and copy the
 * reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder cancoder;

    private final Queue<Double> timestampQueue;

    private final StatusSignal<Double> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveClosedLoopOutput;
    private final StatusSignal<Double> driveCurrent;

    private final StatusSignal<Double> turnAbsolutePosition;
    private final StatusSignal<Double> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<Double> turnVelocity;
    private final StatusSignal<Double> turnAppliedVolts;
    private final StatusSignal<Double> turnClosedLoopOutput;
    private final StatusSignal<Double> turnCurrent;

    // Gear ratios for SDS MK4i L2, adjust as necessary
    private final double DRIVE_GEAR_RATIO = 6.75;
    private final double TURN_GEAR_RATIO = 150.0 / 7.0;

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOTalonFX(int index) {
        switch (index) {
            case 0:
                driveTalon = new TalonFX(0, "drivebase");
                turnTalon = new TalonFX(1, "drivebase");
                cancoder = new CANcoder(2, "drivebase");
                absoluteEncoderOffset = new Rotation2d(0.0); // TODO: Calibrate
                break;
            case 1:
                driveTalon = new TalonFX(3, "drivebase");
                turnTalon = new TalonFX(4, "drivebase");
                cancoder = new CANcoder(5, "drivebase");
                absoluteEncoderOffset = new Rotation2d(0.0); // TODO: Calibrate
                break;
            case 2:
                driveTalon = new TalonFX(6, "drivebase");
                turnTalon = new TalonFX(7, "drivebase");
                cancoder = new CANcoder(8, "drivebase");
                absoluteEncoderOffset = new Rotation2d(0.0); // TODO: Calibrate
                break;
            case 3:
                driveTalon = new TalonFX(9, "drivebase");
                turnTalon = new TalonFX(10, "drivebase");
                cancoder = new CANcoder(11, "drivebase");
                absoluteEncoderOffset = new Rotation2d(0.0); // TODO: Calibrate
                break;
            default:
                throw new RuntimeException("Invalid module index");
        }

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO * Units.inchesToMeters(4 * Math.PI);
        driveConfig.Slot0.kP = 0.05; // % output per m/s of error
        driveConfig.Slot0.kI = 0; // % output per m of integrated error
        driveConfig.Slot0.kD = 0; // % output per m/s^2 of error derivative
        driveConfig.Slot0.kS = 0; // Amps of additional current needed to overcome friction
        driveConfig.Slot0.kV = 0; // Amps of additional current per m/s of velocity
        driveConfig.Slot0.kA = 0; // Amps of additional current per m/s^2 of acceleration
        driveConfig.MotionMagic.MotionMagicAcceleration = 10; // Max allowed acceleration, in m/s^2
        driveConfig.MotionMagic.MotionMagicJerk = 100; // Max allowed jerk, in m/s^3
        driveTalon.getConfigurator().apply(driveConfig);
        setDriveBrakeMode(true);

        var turnConfig = new TalonFXConfiguration();
        turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.Feedback.SensorToMechanismRatio = 1;
        turnConfig.Feedback.RotorToSensorRatio = TURN_GEAR_RATIO;
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        turnConfig.Slot0.kP = 1; // % output per rotation of error
        turnConfig.Slot0.kI = 0; // % output per rotation of integrated error
        turnConfig.Slot0.kD = 0; // % output per rotations/s^2 of error derivative
        turnConfig.Slot0.kS = 0; // Amps of additional current needed to overcome friction
        turnConfig.Slot0.kV = 0; // Amps of additional current per rot/s of velocity
        turnConfig.Slot0.kA = 0; // Amps of additional current per rot/s^2 of acceleration
        turnConfig.MotionMagic.MotionMagicAcceleration = 5; // Max allowed acceleration, in rot/s^2
        turnConfig.MotionMagic.MotionMagicJerk = 50; // Max allowed jerk, in m/s^3
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnTalon.getConfigurator().apply(turnConfig);
        setTurnBrakeMode(true);

        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        drivePosition = driveTalon.getPosition();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveClosedLoopOutput = driveTalon.getClosedLoopOutput();
        driveCurrent = driveTalon.getStatorCurrent();

        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = turnTalon.getPosition();
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnClosedLoopOutput = turnTalon.getClosedLoopOutput();
        turnCurrent = turnTalon.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveClosedLoopOutput,
                driveCurrent,
                turnAbsolutePosition,
                turnVelocity,
                turnAppliedVolts,
                turnClosedLoopOutput,
                turnCurrent);
        driveTalon.optimizeBusUtilization();
        turnTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveClosedLoopOutput,
                driveCurrent,
                turnAbsolutePosition,
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnClosedLoopOutput,
                turnCurrent);

        inputs.drivePositionMeters = Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
        inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble();
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveAppliedDutyCycle = driveClosedLoopOutput.getValueAsDouble();
        inputs.driveCurrentAmps = new double[] { driveCurrent.getValueAsDouble() };

        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
                .minus(absoluteEncoderOffset);
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnAppliedDutyCycle = turnClosedLoopOutput.getValueAsDouble();
        inputs.turnCurrentAmps = new double[] { turnCurrent.getValueAsDouble() };

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
                .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
                .toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void setDriveVelocity(double velocity) {
        driveTalon.setControl(new MotionMagicVelocityTorqueCurrentFOC(velocity));
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void setTurnPosition(Rotation2d position) {
        turnTalon.setControl(new MotionMagicTorqueCurrentFOC(position.getRotations()));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveTalon.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted = isTurnMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turnTalon.getConfigurator().apply(config);
    }
}