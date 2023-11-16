package frc.robot.abstraction.motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
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
    SparkMaxPIDController pid;
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
     * @param motor The CANSparkMax motor object.
     */
    public RevNEO500(CANSparkMax motor) {
        // Load motor.
        this.motor = motor;

        // Spin off configurations in a different thread.
        configureSparkMax(() -> motor.setCANTimeout(0));

        // Clear existing motor configurations and faults.
        factoryDefaults();
        clearStickyFaults();

        // Configure PIDs and encoders.
        this.pid = motor.getPIDController();
        this.encoder = motor.getEncoder();
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
    public void configureIntegratedEncoder(double encoderConversionFactor) {
        if (absoluteEncoder == null) {
            configureSparkMax(() -> encoder.setPositionConversionFactor(encoderConversionFactor));
            configureSparkMax(() -> encoder.setVelocityConversionFactor(encoderConversionFactor / 60));

            // Taken from
            // https://github.com/frc3512/SwerveBot-2022/blob/9d31afd05df6c630d5acb4ec2cf5d734c9093bf8/src/main/java/frc/lib/util/CANSparkMaxUtil.java#L67
            // As we're not using the CAN status frames for alternate encoders, slow down
            // data send rates for alternate encoder values.
            configureCANStatusFrames(10, 20, 20, 500, 500);
        } else {
            configureSparkMax(() -> absoluteEncoder.setPositionConversionFactor(encoderConversionFactor));
            configureSparkMax(() -> absoluteEncoder.setVelocityConversionFactor(encoderConversionFactor));
        }
    }

    @Override
    public void configurePID(double P, double I, double D) {
        configureSparkMax(() -> pid.setP(P));
        configureSparkMax(() -> pid.setI(I));
        configureSparkMax(() -> pid.setD(D));
    }

    @Override
    public void configurePIDWrapping(double minValue, double maxValue) {
        configureSparkMax(() -> pid.setPositionPIDWrappingEnabled(true));
        configureSparkMax(() -> pid.setPositionPIDWrappingMinInput(minValue));
        configureSparkMax(() -> pid.setPositionPIDWrappingMaxInput(maxValue));
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
    public void configureFollow(AbstractMotor otherMotor) {
        if (!(otherMotor instanceof RevNEO500)) {
            throw new UnsupportedOperationException("Leader motor must be of the same motor type!");
        }
        motor.follow((CANSparkMax)otherMotor.getMotor());
    }

    @Override
    public void configureCurrentLimits(double nominalVoltage, int primaryAmpLimit, int secondaryAmpLimit) {
        configureSparkMax(() -> motor.enableVoltageCompensation(nominalVoltage));
        configureSparkMax(() -> motor.setSmartCurrentLimit(primaryAmpLimit));
        configureSparkMax(() -> motor.setSecondaryCurrentLimit(secondaryAmpLimit));
    }

    @Override
    public void configureAbsoluteEncoder(AbstractAbsoluteEncoder encoder) {
        if (encoder.getAbsoluteEncoder() instanceof AbsoluteEncoder) {
            absoluteEncoder = (AbsoluteEncoder) encoder.getAbsoluteEncoder();
            configureSparkMax(() -> pid.setFeedbackDevice(absoluteEncoder));
        }
    }

    @Override
    public void configureRampRate(double rampRate) {
        configureSparkMax(() -> motor.setClosedLoopRampRate(rampRate));
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
    public void setDriveReference(double setPoint, double feedForward) {
        configureSparkMax(() -> pid.setReference(setPoint, ControlType.kVelocity, 0, feedForward));
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
     * Set the CAN status frames, or the clock rate in milliseconds in which certain
     * types of data will be sent down CAN wires.
     *
     * @param CANStatus0 Applied Output, Faults, Sticky Faults, Is Follower
     * @param CANStatus1 Motor Velocity, Motor Temperature, Motor Voltage, Motor
     *                   Current
     * @param CANStatus2 Motor Position
     * @param CANStatus3 Analog Sensor Voltage, Analog Sensor Velocity, Analog
     *                   Sensor Position
     * @param CANStatus4 Alternate Encoder Velocity, Alternate Encoder Position
     */
    public void configureCANStatusFrames(
            int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4) {
        configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0));
        configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1));
        configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2));
        configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3));
        configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4));
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
        DriverStation.reportWarning("Failure configuring NEO 500 on CAN ID " + motor.getDeviceId(), true);
    }
}
