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
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIO;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIOInputsAutoLogged;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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
    return run(() -> io.setTargetAngle(targetPivotSupplier.get()));
  }

  /** Gets a command to manually control the pivot angle. */
  public Command getManualOverrideCommand(DoubleSupplier controlSupplier) {
    return run(() -> io.setRawControl(Volts.of(controlSupplier.getAsDouble() * 12)));
  }

  /** Gets the command to put the pivot in the resting position. */
  public Command getRestingPositionCommand() {
    return getPivotCommand(() -> Rotation2d.fromDegrees(PivotIO.MIN_ANGLE));
  }

  public Rotation2d getAngle() {
    return inputs.angle;
  }

  private SysIdRoutine pivotVelSysIDroutine;

  public Command getPivotVelSysID() {
    return sequence(
        getRestingPositionCommand().until(() -> inputs.isTooFarDown),
        runOnce(
            () ->
                pivotVelSysIDroutine =
                    new SysIdRoutine(
                        new SysIdRoutine.Config(
                            null,
                            null,
                            null,
                            (state) ->
                                Logger.recordOutput(
                                    "Shooter/pivot/velSysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                            (voltage) -> io.setRawControl(voltage), null, this))),
        pivotVelSysIDroutine
            .quasistatic(SysIdRoutine.Direction.kForward)
            .until(() -> inputs.isTooFarUp),
        waitSeconds(1),
        pivotVelSysIDroutine
            .quasistatic(SysIdRoutine.Direction.kReverse)
            .until(() -> inputs.isTooFarDown),
        waitSeconds(1),
        pivotVelSysIDroutine
            .dynamic(SysIdRoutine.Direction.kForward)
            .until(() -> inputs.isTooFarUp),
        waitSeconds(1),
        pivotVelSysIDroutine
            .dynamic(SysIdRoutine.Direction.kReverse)
            .until(() -> inputs.isTooFarDown));
  }
}
