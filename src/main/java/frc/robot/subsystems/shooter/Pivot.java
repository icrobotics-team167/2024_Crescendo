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
        leaderMotor.configureEncoder(360); // TODO: Configure
        leaderMotor.configureAbsoluteEncoder(encoder, 360);
        leaderMotor.setPosition(encoder.getAbsolutePosition().getDegrees());
        leaderMotor.configureMotorBrake(true);
        leaderMotor.configurePID(0, 0, 0); // TODO: Configure

        followerMotor.configureCurrentLimits(followerMotor.getNominalVoltage(), followerMotor.getPrimaryCurrentLimit(),
                followerMotor.getSecondaryCurrentLimit());
        followerMotor.configureFollow(followerMotor, true);

        this.encoder = encoder;
        motor = leaderMotor;
    }

    public void setDesiredAngle(Rotation2d desired) {
        Telemetry.sendNumber("Pivot/desiredAngle", desired.getDegrees(), Verbosity.HIGH);
        motor.setTurnReference(desired);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(motor.getPosition());
    }
}
