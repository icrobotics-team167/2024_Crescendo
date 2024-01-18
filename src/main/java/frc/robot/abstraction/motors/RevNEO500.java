package frc.robot.abstraction.motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.abstraction.encoders.AbstractAbsoluteEncoder;
import frc.robot.Constants.Robot.Motors.Neo500;
import java.util.function.Supplier;

/**
 * Class to represent a REV NEO 500 motor.
 */
public class RevNEO500 extends AbstractMotor {

    /**
     * The motor itself.
     */
    CANSparkMax motor;
    /**
     * The PID controller for the motor.
     */
    SparkPIDController pid;
    /**
     * The motor's internal encoder.
     */
    RelativeEncoder encoder;
    /**
     * The motor's absolute encoder, if there is one.
     */
    AbsoluteEncoder absoluteEncoder;
    /**
     * If the motor controller has already reset to factory refaults. If true,
     * factoryDefaults() does nothing.
     */
    boolean hasFactoryReset;
    /**
     * Feedforward component of the motor's control loop.
     */
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    /**
     * Constructs a new REV NEO 500 motor.
     * 
     * @param CANID The CAN ID of the Spark Max controlling the motor.
     */
    public RevNEO500(int CANID) {
        this(new CANSparkMax(CANID, MotorType.kBrushless));
    }

    /**
     * Constructs a new REV NEO 500 motor.
     * 
     * @param sparkMax The CANSparkMax motor object.
     */
    public RevNEO500(CANSparkMax sparkMax) {
        // Load motor.
        this.motor = sparkMax;

        // Spin off configurations in a different thread.
        configureSparkMax(() -> sparkMax.setCANTimeout(0));

        // Clear existing motor configurations and faults.
        factoryDefaults();
        clearStickyFaults();

        // Configure PIDs and encoders.
        this.pid = sparkMax.getPIDController();
        this.encoder = sparkMax.getEncoder();
        this.pid.setFeedbackDevice(encoder);
    }

    @Override
    public void factoryDefaults() {
        if (!hasFactoryReset) {
            configureSparkMax(motor::restoreFactoryDefaults);
            hasFactoryReset = true;
        }
    }

    @Override
    public void clearStickyFaults() {
        configureSparkMax(motor::clearFaults);
    }

    @Override
    public void configureEncoder(double encoderConversionFactor) {
        if (absoluteEncoder == null) {
            configureSparkMax(() -> encoder.setPositionConversionFactor(encoderConversionFactor));
            configureSparkMax(() -> encoder.setVelocityConversionFactor(encoderConversionFactor / 60));
        } else {
            configureSparkMax(() -> absoluteEncoder.setPositionConversionFactor(encoderConversionFactor));
            configureSparkMax(() -> absoluteEncoder.setVelocityConversionFactor(encoderConversionFactor / 60));
        }
    }

    @Override
    public void configurePID(double P, double I, double D) {
        configureSparkMax(() -> pid.setP(P));
        configureSparkMax(() -> pid.setI(I));
        configureSparkMax(() -> pid.setD(D));
    }

    @Override
    public void configurePIDWrapping(boolean wrapPID) {
        configureSparkMax(() -> pid.setPositionPIDWrappingEnabled(wrapPID));
        configureSparkMax(() -> pid.setPositionPIDWrappingMinInput(-180));
        configureSparkMax(() -> pid.setPositionPIDWrappingMaxInput(180));
    }

    @Override
    public void configureMotorBrake(boolean brake) {
        configureSparkMax(() -> motor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast));
    }

    @Override
    public void configureInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    @Override
    public void configureFollow(AbstractMotor otherMotor, boolean invert) {
        if (!(otherMotor instanceof RevNEO500)) {
            throw new UnsupportedOperationException("Leader motor must be of the same motor type!");
        }
        motor.follow((CANSparkMax) otherMotor.getMotor(), invert);
    }

    @Override
    public void configureCurrentLimits(double nominalVoltage, int primaryAmpLimit, int secondaryAmpLimit) {
        configureSparkMax(() -> motor.enableVoltageCompensation(nominalVoltage));
        configureSparkMax(() -> motor.setSmartCurrentLimit(primaryAmpLimit));
        configureSparkMax(() -> motor.setSecondaryCurrentLimit(secondaryAmpLimit));
    }

    @Override
    public void configureAbsoluteEncoder(AbstractAbsoluteEncoder encoder, double positionConversionFactor) {
        if (encoder.getAbsoluteEncoder() instanceof AbsoluteEncoder) {
            absoluteEncoder = (AbsoluteEncoder) encoder.getAbsoluteEncoder();
            configureEncoder(positionConversionFactor);
            absoluteEncoder.setZeroOffset(encoder.getOffset().getDegrees());
            configureSparkMax(() -> pid.setFeedbackDevice(absoluteEncoder));
        }
    }

    @Override
    public void configureRampRate(double rampRate) {
        configureSparkMax(() -> motor.setClosedLoopRampRate(rampRate));
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
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setVelocityReference(double setPoint) {
        configureSparkMax(() -> pid.setReference(setPoint, ControlType.kVelocity, 0, feedforward.calculate(setPoint)));
    }

    @Override
    public void setTurnReference(Rotation2d setPoint) {
        configureSparkMax(() -> pid.setReference(setPoint.getDegrees(), ControlType.kPosition, 0, 0));
    }

    @Override
    public void setPosition(double position) {
        if (absoluteEncoder != null) {
            absoluteEncoder.setZeroOffset(absoluteEncoder.getPosition() - position);
        } else {
            configureSparkMax(() -> encoder.setPosition(position));
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
        return encoder.getVelocity();
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getNominalVoltage() {
        return Neo500.CurrentLimits.NOMINAL_VOLTAGE;
    }

    @Override
    public int getPrimaryCurrentLimit() {
        return Neo500.CurrentLimits.PRIMARY_CURRENT_LIMIT;
    }

    @Override
    public int getSecondaryCurrentLimit() {
        return Neo500.CurrentLimits.SECONDARY_CURRENT_LIMIT;
    }

    @Override
    public double getMaxRPM() {
        return Neo500.MAX_RPM;
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
    private void configureSparkMax(Supplier<REVLibError> config) {
        for (int i = 0; i < maximumRetries; i++) {
            if (config.get() == REVLibError.kOk) {
                return;
            }
        }
        DriverStation.reportWarning("Failure configuring Spark Max on CAN ID " + motor.getDeviceId(), true);
    }
}
