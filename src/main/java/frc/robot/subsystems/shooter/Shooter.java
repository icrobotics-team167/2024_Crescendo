package frc.robot.subsystems.shooter;

import frc.robot.abstraction.motors.AbstractMotor;

public class Shooter {
    AbstractMotor motor;

    public Shooter(AbstractMotor leaderMotor, AbstractMotor followerMotor) {
        leaderMotor.configureCurrentLimits(leaderMotor.getNominalVoltage(), 150, 200);
        followerMotor.configureCurrentLimits(followerMotor.getNominalVoltage(), 150, 200);
        leaderMotor.configureMotorBrake(false);
        followerMotor.configureMotorBrake(false);

        followerMotor.configureFollow(leaderMotor, true);

        motor = leaderMotor;
    }

    public void run() {
        motor.set(1);
    }

    public void stop() {
        motor.stop();
    }

    public double getVelocityPercentage() {
        return motor.getVelocity() / motor.getMaxRPM();
    }
}
