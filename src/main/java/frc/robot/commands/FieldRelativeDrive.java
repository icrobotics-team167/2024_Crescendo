package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving.Deadbands;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class FieldRelativeDrive extends Command {
    private SwerveSubsystem drivebase;
    private DoubleSupplier xInput;
    private DoubleSupplier yInput;
    private DoubleSupplier rotInput;

    public FieldRelativeDrive(SwerveSubsystem drivebase, DoubleSupplier xInput, DoubleSupplier yInput,
            DoubleSupplier rotInput) {
        this.drivebase = drivebase;
        this.xInput = xInput;
        this.yInput = yInput;
        this.rotInput = rotInput;

        setName("Field Relative Drive");
        addRequirements(drivebase);
    }

    @Override
    public void execute() {
        double x = MathUtil.applyDeadband(xInput.getAsDouble(), Deadbands.PRIMARY_LEFT);
        double y = MathUtil.applyDeadband(yInput.getAsDouble(), Deadbands.PRIMARY_LEFT);
        double rot = MathUtil.applyDeadband(rotInput.getAsDouble(), Deadbands.PRIMARY_RIGHT);

        double controlMagnitude = Math.hypot(x, y);
        x /= Math.max(controlMagnitude, 1);
        x /= Math.max(controlMagnitude, 1);

        boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        drivebase.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                x * drivebase.getMaxLinearSpeedMetersPerSec(),
                y * drivebase.getMaxLinearSpeedMetersPerSec(),
                rot * drivebase.getMaxAngularSpeedRadPerSec(),
                isFlipped ? drivebase.getRotation().plus(new Rotation2d(Math.PI)) : drivebase.getRotation()));
    }
}
