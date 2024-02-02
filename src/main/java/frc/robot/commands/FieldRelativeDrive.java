// Copyright (c) 2024 FRC 167
// https://www.thebluealliance.com/team/167
// https://github.com/icrobotics-team167
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.Constants.Driving.Deadbands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.MathUtils;
import java.util.function.DoubleSupplier;

public class FieldRelativeDrive extends Command {
  private SwerveSubsystem drivebase;
  private DoubleSupplier xInput;
  private DoubleSupplier yInput;
  private DoubleSupplier rotInput;

  public FieldRelativeDrive(
      SwerveSubsystem drivebase,
      DoubleSupplier xInput,
      DoubleSupplier yInput,
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
    double x =
        MathUtils.inOutDeadband(
            xInput.getAsDouble(),
            Deadbands.PRIMARY_LEFT_INNER,
            Deadbands.PRIMARY_LEFT_OUTER,
            Driving.PRIMARY_DRIVER_EXPONENT);
    double y =
        MathUtils.inOutDeadband(
            yInput.getAsDouble(),
            Deadbands.PRIMARY_LEFT_INNER,
            Deadbands.PRIMARY_LEFT_OUTER,
            Driving.PRIMARY_DRIVER_EXPONENT);
    double rot =
        MathUtils.inOutDeadband(
            rotInput.getAsDouble(),
            Deadbands.PRIMARY_RIGHT_INNER,
            Deadbands.PRIMARY_RIGHT_OUTER,
            Driving.PRIMARY_DRIVER_EXPONENT);

    double controlMagnitude = Math.hypot(x, y);
    x /= Math.max(controlMagnitude, 1);
    x /= Math.max(controlMagnitude, 1);

    drivebase.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x * drivebase.getMaxLinearVelocity().in(MetersPerSecond),
            y * drivebase.getMaxLinearVelocity().in(MetersPerSecond),
            rot * drivebase.getMaxAngularVelocity().in(RadiansPerSecond),
            drivebase.getRotation()));
  }
}
