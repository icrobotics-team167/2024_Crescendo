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
  private PivotIOInputsAutoLogged inputs;

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
    return run(() -> io.setPivotControl(Volts.of(controlSupplier.getAsDouble() * 12)));
  }

  /** Gets the command to put the pivot in the resting position. */
  public Command getRestingPositionCommand() {
    return getPivotCommand(() -> Rotation2d.fromDegrees(15));
  }

  /** The sysid routine generator. */
  private SysIdRoutine sysIDRoutine;

  /** Gets the command to run system identification on the pivot. */
  public Command getSysID() {
    return sequence(
        runOnce(
            () ->
                sysIDRoutine =
                    new SysIdRoutine(
                        new SysIdRoutine.Config(
                            null,
                            null,
                            null,
                            (state) ->
                                Logger.recordOutput("Shooter/pivot/SysIDState", state.toString())),
                        new SysIdRoutine.Mechanism(
                            (voltage) -> io.setPivotControl(voltage), null, this))),
        sysIDRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> inputs.isTooFarUp),
        runOnce(io::stop),
        waitSeconds(2),
        sysIDRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> inputs.isTooFarDown),
        runOnce(io::stop),
        waitSeconds(2),
        sysIDRoutine.dynamic(SysIdRoutine.Direction.kForward).until(() -> inputs.isTooFarUp),
        runOnce(io::stop),
        waitSeconds(2),
        sysIDRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> inputs.isTooFarDown));
  }
}