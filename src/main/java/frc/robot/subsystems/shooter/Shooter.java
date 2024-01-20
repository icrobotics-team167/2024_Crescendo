package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.abstraction.motors.AbstractMotor;

public class Shooter {
    AbstractMotor motor;
    double targetSpeed;

    public Shooter(AbstractMotor leaderMotor, AbstractMotor followerMotor, double targetSpeed) {
        leaderMotor.configureCurrentLimits(leaderMotor.getNominalVoltage(), 150, 200);
        followerMotor.configureCurrentLimits(followerMotor.getNominalVoltage(), 150, 200);
        leaderMotor.configureMotorBrake(false);
        followerMotor.configureMotorBrake(false);

        leaderMotor.configureEncoder(Units.inchesToMeters(4.0 * Math.PI));
        leaderMotor.configurePID(0, 0, 0); // TODO: Tune
        leaderMotor.configureFeedForward(0, 0, 0);

        followerMotor.configureFollow(leaderMotor, true); //might need to make sure follower and leader are reversed directions

        motor = leaderMotor;

        this.targetSpeed = targetSpeed;
    }

    public void runTest() {
        motor.setVelocityReference(30);
    }

    public void run(){
        if(this.getVelocity() < targetSpeed) {
            motor.set(1);
        }
        if(this.getVelocity() >= targetSpeed) {
            motor.stop(); //make sure its coast mode like a coooooool dude
        }
    }  //Bang Bang controller thing, this may implode

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
}
