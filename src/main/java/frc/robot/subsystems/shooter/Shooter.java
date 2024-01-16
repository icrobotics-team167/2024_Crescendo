package frc.robot.subsystems.shooter;

import frc.robot.abstraction.motors.AbstractMotor;

public class Shooter {
    AbstractMotor motor;

    public Shooter(AbstractMotor leaderMotor, AbstractMotor followerMotor) {
        leaderMotor.configureCurrentLimits(leaderMotor.getNominalVoltage(), 150, 200);
        followerMotor.configureCurrentLimits(followerMotor.getNominalVoltage(), 150, 200);

        followerMotor.configureFollow(leaderMotor, true);

        motor = leaderMotor;
    }

    public void run() {
        motor.set(1);
    }

    public void stop() {
        motor.set(0);
    }
}
