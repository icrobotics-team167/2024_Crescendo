package frc.robot.abstraction.motors;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Robot.Motors.Vortex;
import frc.robot.abstraction.encoders.AbstractAbsoluteEncoder;

/**
 * Class to represent a Rev NEO Vortex.
 */
public class RevNEOVortex extends AbstractMotor {
    /**
     * The motor itself.
     */
    CANSparkFlex motor;
    /**
     * The motor's absolute encoder, if there is one.
     */
    AbsoluteEncoder absoluteEncoder;
    /**
     * If the motor controller has already reset to factory refaults. If true,
     * factoryDefaults() does nothing.
     */
    boolean hasFactoryReset = false;
    /**
     * Feedforward component of the motor's control loop.
     */
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    /**
     * Constructs a new Rev NEO Vortex motor.
     * 
     * @param CANID The CAN ID of the motor.
     */
    public RevNEOVortex(int CANID) {
        motor = new CANSparkFlex(CANID, MotorType.kBrushless);
    }

    @Override
    public void factoryDefaults() {
        if (hasFactoryReset) {
            return;
        }
        configureSparkFlex(() -> motor.restoreFactoryDefaults());
        hasFactoryReset = true;
    }

    @Override
    public void clearStickyFaults() {
        configureSparkFlex(() -> motor.clearFaults());
    }

    @Override
    public void configureEncoder(double positionConversionFactor) {
        if (absoluteEncoder == null) {
            configureSparkFlex(() -> motor.getEncoder().setPositionConversionFactor(positionConversionFactor));
            configureSparkFlex(() -> motor.getEncoder().setVelocityConversionFactor(positionConversionFactor / 60));
        } else {
            configureSparkFlex(() -> absoluteEncoder.setPositionConversionFactor(positionConversionFactor));
            configureSparkFlex(() -> absoluteEncoder.setVelocityConversionFactor(positionConversionFactor / 60));
        }
    }

    @Override
    public void configurePID(double P, double I, double D) {
        configureSparkFlex(() -> motor.getPIDController().setP(P));
        configureSparkFlex(() -> motor.getPIDController().setP(I));
        configureSparkFlex(() -> motor.getPIDController().setP(D));
    }

    @Override
    public void configurePIDWrapping(boolean wrapPID) {
        configureSparkFlex(() -> motor.getPIDController().setPositionPIDWrappingEnabled(wrapPID));
        configureSparkFlex(() -> motor.getPIDController().setPositionPIDWrappingMaxInput(180));
        configureSparkFlex(() -> motor.getPIDController().setPositionPIDWrappingMinInput(-180));
    }

    @Override
    public void configureMotorBrake(boolean brake) {
        configureSparkFlex(() -> motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast));
    }

    @Override
    public void configureInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    @Override
    public void configureFollow(AbstractMotor otherMotor, boolean invert) {
        if (!(otherMotor instanceof RevNEOVortex)) {
            throw new UnsupportedOperationException("Leader motor must be of the same motor type!");
        }
        motor.follow((CANSparkFlex) otherMotor.getMotor(), invert);
    }

    @Override
    public void configureCurrentLimits(double nominalVoltage, int primaryAmpLimit, int secondaryAmpLimit) {
        configureSparkFlex(() -> motor.enableVoltageCompensation(nominalVoltage));
        configureSparkFlex(() -> motor.setSmartCurrentLimit(primaryAmpLimit));
        configureSparkFlex(() -> motor.setSecondaryCurrentLimit(secondaryAmpLimit));
    }

    @Override
    public void configureAbsoluteEncoder(AbstractAbsoluteEncoder absoluteEncoder, double positionConversionFactor) {
        if (!(absoluteEncoder.getAbsoluteEncoder() instanceof AbsoluteEncoder)) {
            throw new UnsupportedOperationException("Absolute encoder must be a Rev supported encoder!");
        }
        this.absoluteEncoder = (AbsoluteEncoder) (absoluteEncoder.getAbsoluteEncoder());
        configureEncoder(positionConversionFactor);
        this.absoluteEncoder.setZeroOffset(absoluteEncoder.getOffset().getDegrees());
    }

    @Override
    public void configureRampRate(double rampRate) {
        configureSparkFlex(() -> motor.setClosedLoopRampRate(rampRate));
        configureSparkFlex(() -> motor.setOpenLoopRampRate(rampRate));
    }

    @Override
    public void configureFeedForward(double kS, double kV, double kA) {
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Override
    public void set(double setPoint) {
        motor.set(setPoint);
    }

    @Override
    public void setVolts(double voltsSetPoint) {
        motor.setVoltage(voltsSetPoint);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setVelocityReference(double setPoint) {
        motor.getPIDController().setReference(setPoint, ControlType.kVelocity, 0, feedforward.calculate(setPoint));
    }

    @Override
    public void setTurnReference(Rotation2d setPoint) {
        motor.getPIDController().setReference(setPoint.getDegrees(), ControlType.kPosition);
    }

    @Override
    public void setPosition(double position) {
        motor.getEncoder().setPosition(position);
        if (absoluteEncoder != null) {
            absoluteEncoder.setZeroOffset(absoluteEncoder.getPosition() - position);
        }
    }

    @Override
    public Object getMotor() {
        return motor;
    }

    @Override
    public boolean isAttachedAbsoluteEncoder() {
        return absoluteEncoder != null;
    }

    @Override
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getNominalVoltage() {
        return Vortex.CurrentLimits.NOMINAL_VOLTAGE;
    }

    @Override
    public int getPrimaryCurrentLimit() {
        return Vortex.CurrentLimits.PRIMARY_CURRENT_LIMIT;
    }

    @Override
    public int getSecondaryCurrentLimit() {
        return Vortex.CurrentLimits.SECONDARY_CURRENT_LIMIT;
    }

    @Override
    public double getMaxRPM() {
        return Vortex.MAX_RPM;
    }

    /**
     * <p>
     * Function to configure the Spark Max motor controller. Attempts to configure
     * it a couple times, but if it fails on all attempts, throws an error.
     * 
     * <p>
     * Usage example:
     * 
     * <pre>
     * configureSparkMax(() -> motor.enableVoltageCompensation(nominalVoltage));
     * </pre>
     * 
     * @param config The configuration command.
     */
    private void configureSparkFlex(Supplier<REVLibError> config) {
        for (int i = 0; i < maximumRetries; i++) {
            if (config.get() == REVLibError.kOk) {
                return;
            }
        }
        DriverStation.reportWarning("Failure configuring Spark Max on CAN ID " + motor.getDeviceId(), true);
    }
}
