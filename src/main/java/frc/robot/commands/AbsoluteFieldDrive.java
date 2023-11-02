package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Robot.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * A command to drive the swerve drivebase.
 */
public class AbsoluteFieldDrive extends CommandBase {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY, vRot;

    /**
     * A command to drive the drivebase.
     * 
     * @param swerve The swerve drivebase object.
     * @param vX     A supplier providing a double. +1 is full speed away from the
     *               driver station, -1 is full speed away from the driver station.
     * @param vY     A supplier providing a double. +1 is full speed left relative
     *               to the driver station, -1 is full speed right relative to the
     *               driver station.
     * @param vRot   A supplier providing a double. +1 is full speed CCW, -1 is full
     *               speed CW.
     */
    public AbsoluteFieldDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vRot) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.vRot = vRot;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double xVel = vX.getAsDouble() * SwerveDrive.MAX_TRANSLATIONAL_VEL;
        double yVel = vY.getAsDouble() * SwerveDrive.MAX_TRANSLATIONAL_VEL;
        double rotVel = vRot.getAsDouble() * SwerveDrive.MAX_ROTATIONAL_VEL;
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xVel, yVel, rotVel);
        swerve.drive(desiredSpeeds, true);
    }
}