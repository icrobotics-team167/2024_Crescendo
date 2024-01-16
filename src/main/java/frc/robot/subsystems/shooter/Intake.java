package frc.robot.subsystems.shooter;

import frc.robot.abstraction.motors.AbstractMotor;

public class Intake {
    AbstractMotor motor;

    public Intake(AbstractMotor motor) {
        motor.configureCurrentLimits(motor.getNominalVoltage(), motor.getPrimaryCurrentLimit(),
                motor.getSecondaryCurrentLimit());
        this.motor = motor;
    }

    public void run() {
        motor.set(1);
    }

    public void stop() {
        motor.set(0);
    }
}
