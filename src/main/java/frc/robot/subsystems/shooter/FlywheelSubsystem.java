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

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.interfaceLayers.FlywheelIO;
import frc.robot.subsystems.shooter.interfaceLayers.FlywheelIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {
  private final FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/flywheel", inputs);
  }

  /** Gets the command to spin up the flywheel. Stops spinning when the command ends. */
  public Command getSpinCommand() {
    return run(() -> io.run()).finallyDo(() -> io.stop());
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
                            (voltage) -> io.runVoltage(voltage), null, this))),
        sysIDRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        runOnce(io::stop),
        waitSeconds(2),
        sysIDRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        runOnce(io::stop),
        waitSeconds(2),
        sysIDRoutine.dynamic(SysIdRoutine.Direction.kForward),
        runOnce(io::stop),
        waitSeconds(2),
        sysIDRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }
}
