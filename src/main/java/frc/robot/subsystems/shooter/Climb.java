package frc.robot.subsystems.shooter;

//import edu.wpi.first.math.util.Units;
import frc.robot.abstraction.motors.AbstractMotor;

public class Climb {
    AbstractMotor motor;

    public Climb(AbstractMotor motor){
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