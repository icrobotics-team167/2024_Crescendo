package frc.robot.abstraction.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Robot.Motors.Neo550;

public class RevNEO550 extends RevNEO500 {
    /**
     * Constructs a new REV NEO 550 motor.
     * 
     * @param CANID The CAN ID of the Spark Max controlling the motor.
     */
    public RevNEO550(int CANID) {
        this(new CANSparkMax(CANID, MotorType.kBrushless));
    }

    /**
     * Constructs a new REV NEO 550 motor.
     * 
     * @param sparkMax The CANSparkMax motor object.
     */
    public RevNEO550(CANSparkMax sparkMax) {
        super(sparkMax);
    }

    @Override
    public double getNominalVoltage() {
        return Neo550.CurrentLimits.NOMINAL_VOLTAGE;
    }

    @Override
    public int getPrimaryCurrentLimit() {
        return Neo550.CurrentLimits.PRIMARY_CURRENT_LIMIT;
    }

    @Override
    public int getSecondaryCurrentLimit() {
        return Neo550.CurrentLimits.SECONDARY_CURRENT_LIMIT;
    }

    @Override
    public double getMaxRPM() {
        return Neo550.MAX_RPM;
    }
}