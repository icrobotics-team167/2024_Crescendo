package frc.robot.subsystems.shooter;

import frc.robot.abstraction.motors.AbstractMotor;

public class Intake {
    AbstractMotor feedMotor;
    AbstractMotor intakeMotor;

    public Intake(AbstractMotor motor) {
        motor.configureCurrentLimits(motor.getNominalVoltage(), motor.getPrimaryCurrentLimit(),
                motor.getSecondaryCurrentLimit());
        this.intakeMotor = motor;

    }

    public void run() {
        intakeMotor.set(1);
    }

    public void stop() {
        intakeMotor.stop();
    }
}
