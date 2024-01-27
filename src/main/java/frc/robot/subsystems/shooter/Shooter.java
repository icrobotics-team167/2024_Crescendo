package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import frc.robot.abstraction.motors.AbstractMotor;

public class Shooter {
    AbstractMotor motor;
    double targetSpeed;

    public Shooter(AbstractMotor leaderMotor, AbstractMotor followerMotor, double targetSpeed) {
        leaderMotor.configureCurrentLimits(leaderMotor.getNominalVoltage(), 80, 90);
        followerMotor.configureCurrentLimits(followerMotor.getNominalVoltage(), 80, 90);
        leaderMotor.configureMotorBrake(false);
        followerMotor.configureMotorBrake(false);

        leaderMotor.configureEncoder(Units.inchesToMeters(4.0 * Math.PI));
        leaderMotor.configurePID(0.25, 0, 0); // TODO: Tune
        leaderMotor.configureFeedForward(0, 0, 0);

        followerMotor.configureFollow(leaderMotor, true); // might need to make sure follower and leader are reversed
                                                          // directions

        motor = leaderMotor;

        this.targetSpeed = targetSpeed;
    }

    // TODO: Figure out design of shooter to finalize methods

    public void runTest() {
        // Bang Bang controller thing, this may implode
        if (getVelocity() < targetSpeed) {
            motor.set(1);
        }
        if (getVelocity() >= targetSpeed) {
            motor.stop(); // make sure its coast mode like a coooooool dude
        }
    }

    public void run() {
        motor.setVelocityReference(targetSpeed);
    }

    public void runVolts(double volts) {
        motor.setVolts(volts);
    }

    public double getPosition() {
        return motor.getPosition();
    }

    public void stop() {
        motor.stop();
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public double getTargetVelocity() {
        return motor.getMaxRPM() * Units.inchesToMeters(4.0 * Math.PI);
    }
}
