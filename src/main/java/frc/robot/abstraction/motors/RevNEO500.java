package frc.robot.abstraction.motors;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.abstraction.encoders.AbstractAbsoluteEncoder;
import frc.robot.Constants.Robot.Motors.Neo500;

public class RevNEO500 extends AbstractMotor {

    CANSparkMax motor;

    SparkMaxPIDController pid;

    RelativeEncoder encoder;

    AbsoluteEncoder absoluteEncoder;

    boolean hasFactoryReset;

    public RevNEO500(int CANID) {
        this(new CANSparkMax(CANID, MotorType.kBrushless));
    }

    public RevNEO500(CANSparkMax motor) {
        this.motor = motor;

        factoryDefaults();
        clearStickyFaults();

        this.pid = motor.getPIDController();
        this.encoder = motor.getEncoder();
        this.pid.setFeedbackDevice(encoder);

        // Spin off configurations in a different thread.
        configureSparkMax(() -> motor.setCANTimeout(0));
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
    public void configureIntegratedEncoder(double positionConversionFactor) {
        if (absoluteEncoder == null) {
            configureSparkMax(() -> encoder.setPositionConversionFactor(positionConversionFactor));
            configureSparkMax(() -> encoder.setVelocityConversionFactor(positionConversionFactor / 60));

            // Taken from
            // https://github.com/frc3512/SwerveBot-2022/blob/9d31afd05df6c630d5acb4ec2cf5d734c9093bf8/src/main/java/frc/lib/util/CANSparkMaxUtil.java#L67
            configureCANStatusFrames(10, 20, 20, 500, 500);
        } else {
            configureSparkMax(() -> absoluteEncoder.setPositionConversionFactor(positionConversionFactor));
            configureSparkMax(() -> absoluteEncoder.setVelocityConversionFactor(positionConversionFactor));
        }
    }

    @Override
    public void configrePID(double P, double I, double D) {
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
    public void configureCurrentLimits(double nominalVoltage, int primaryAmpLimit, int secondaryAmpLimit) {
        configureSparkMax(() -> motor.enableVoltageCompensation(nominalVoltage));
        configureSparkMax(() -> motor.setSmartCurrentLimit(primaryAmpLimit));
        configureSparkMax(() -> motor.setSecondaryCurrentLimit(secondaryAmpLimit));
    }

    @Override
    public void configureAbsoluteEncoder(AbstractAbsoluteEncoder encoder, double offset) {
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
     * Set the CAN status frames.
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
        // TODO: Configure Status Frame 5 and 6 if necessary
        // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
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
