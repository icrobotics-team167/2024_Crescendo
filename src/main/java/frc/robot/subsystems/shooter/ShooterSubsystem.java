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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.interfaceLayers.FlywheelIO;
import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIO;
import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIOInputsAutoLogged;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIO;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIOInputsAutoLogged;
import frc.robot.subsystems.shooter.interfaceLayers.ShooterIOInputsAutoLogged;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final PivotIO pivot;
  private PivotIOInputsAutoLogged pivotInputs;

  private final FlywheelIO flywheel;
  private ShooterIOInputsAutoLogged shooterInputs;

  private final NoteDetectorIO noteDetector;
  private NoteDetectorIOInputsAutoLogged noteDetectorInputs;

  public ShooterSubsystem(PivotIO pivot, FlywheelIO flywheel, NoteDetectorIO noteDetector) {
    this.pivot = pivot;
    this.flywheel = flywheel;
    this.noteDetector = noteDetector;
    setDefaultCommand(getNeutralPositionCommand());
  }

  @Override
  public void periodic() {
    pivot.updateInputs(pivotInputs);
    Logger.processInputs("Shooter/pivot", pivotInputs);
    flywheel.updateInputs(shooterInputs);
    Logger.processInputs("Shooter/flywheel", shooterInputs);
    noteDetector.updateInputs(noteDetectorInputs);
    Logger.processInputs("Shooter/noteDetector", noteDetectorInputs);
  }

  public Command getNeutralPositionCommand() {
    return run(
        () -> {
          flywheel.stop();
          pivot.setTargetAngle(Degrees.of(30)); // TODO: Figure out lowest angle
        });
  }

  /**
   * Gets the command to manually control the shooter. Joystick input suppliers should already have
   * deadbands applied.
   */
  public Command getManualAimCommand(DoubleSupplier pivotInput) {
    return run(() -> pivot.setPivotControl(Volts.of(pivotInput.getAsDouble() * 12)));
  }
}
