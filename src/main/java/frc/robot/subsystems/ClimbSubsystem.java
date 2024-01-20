package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstraction.motors.AbstractMotor;
import frc.robot.abstraction.motors.AbstractMotor;
import frc.robot.abstraction.motors.RevNEO500;

public class ClimbSubsystem extends SubsystemBase {
        AbstractMotor motor;

    public ClimbSubsystem(AbstractMotor motor){
        motor.configureCurrentLimits(motor.getNominalVoltage(), motor.getPrimaryCurrentLimit(), motor.getSecondaryCurrentLimit());
        this.motor = motor;
    }

    public void runForward(){
        this.motor.set(1);
    }

    public void runBackward(){
        this.motor.set(-1);
    }

    public void stop(){
        this.motor.stop();
    }
}
