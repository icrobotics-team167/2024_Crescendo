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

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIO;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIOInputsAutoLogged;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {
  private final PivotIO io;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public PivotSubsystem(PivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/pivot", inputs);
  }

  /**
   * Gets a command to set the target angle for the pivot mechanism. 0 degrees is parallel to the
   * ground.
   */
  public Command getPivotCommand(Supplier<Rotation2d> targetPivotSupplier) {
    return run(() -> io.setTargetAngle(targetPivotSupplier.get()))
        .finallyDo(() -> io.setVelocityControl(RPM.of(0)))
        .withName("Pivot to angle");
  }

  /** Gets a command to manually control the pivot angle. */
  public Command getManualOverrideCommand(DoubleSupplier controlSupplier) {
    return run(() -> io.setVelocityControl(DegreesPerSecond.of(controlSupplier.getAsDouble() * 45)))
        .finallyDo(() -> io.setVelocityControl(DegreesPerSecond.of(0)));
  }

  /** Gets the command to put the pivot in the resting position. */
  public Command getRestingPositionCommand() {
    return getPivotCommand(() -> Rotation2d.fromDegrees(PivotIO.MIN_ANGLE));
  }

  public Rotation2d getAngle() {
    return inputs.angle;
  }

  @AutoLogOutput
  public boolean isAtSetpoint() {
    return Math.abs(inputs.angleSetpoint.getDegrees() - inputs.angle.getDegrees()) < 0.2;
  }
}
