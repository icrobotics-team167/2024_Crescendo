package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.abstraction.encoders.AbstractAbsoluteEncoder;
import frc.robot.abstraction.motors.AbstractMotor;
import frc.robot.helpers.Telemetry;
import frc.robot.helpers.Telemetry.Verbosity;

public class Pivot {
    AbstractMotor motor;
    AbstractAbsoluteEncoder encoder;

    public Pivot(AbstractMotor leaderMotor, AbstractMotor followerMotor, AbstractAbsoluteEncoder encoder) {
        leaderMotor.configureCurrentLimits(leaderMotor.getNominalVoltage(), leaderMotor.getPrimaryCurrentLimit(),
                leaderMotor.getSecondaryCurrentLimit());
        leaderMotor.configureEncoder(0.9); // TODO: Configure
        // leaderMotor.configureAbsoluteEncoder(encoder, 360);
        // leaderMotor.setPosition(encoder.getAbsolutePosition().getDegrees());
        leaderMotor.setPosition(60);
        leaderMotor.configureMotorBrake(true);
        leaderMotor.configurePID(1.0 / 45.0, 0, 0); // TODO: Configure
        leaderMotor.configureFeedForward(0, 0, 0);

        followerMotor.configureCurrentLimits(followerMotor.getNominalVoltage(), followerMotor.getPrimaryCurrentLimit(),
                followerMotor.getSecondaryCurrentLimit());
        followerMotor.configureFollow(followerMotor, true);

        this.encoder = encoder;
        motor = leaderMotor;
    }

    // TODO: Figure out the design of the pivot to finalize methods

    public void setDesiredAngle(Rotation2d desired) {
        Telemetry.sendNumber("Pivot/desiredAngle", desired.getDegrees(), Verbosity.HIGH);
        motor.setTurnReference(desired);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(motor.getPosition());
    }

    public void move(double speed) {
        motor.set(speed);
    }
}
