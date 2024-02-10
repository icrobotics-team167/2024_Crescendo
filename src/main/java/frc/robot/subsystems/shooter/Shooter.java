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
import frc.robot.subsystems.shooter.interfaceLayers.IntakeIO;
import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIO;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIO;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIOSparkFlex;
import java.util.function.DoubleSupplier;

/** A class containing all the logic and commands to make the shooter mechanism work. */
public class Shooter {
  // private final FlywheelSubsystem flywheel;
  private final PivotSubsystem pivot;
  private final NoteDetectorSubsystem noteDetector;
  private final IntakeSubsystem intake;

  public Shooter(PivotIO pivotIO, NoteDetectorIO noteDetectorIO, IntakeIO intakeIO) {
    // TODO: Implement flywheel and pivot interfaces
    // flywheel = new FlywheelSubsystem(null);
    pivot = new PivotSubsystem(new PivotIOSparkFlex());
    noteDetector = new NoteDetectorSubsystem(noteDetectorIO);
    intake = new IntakeSubsystem(intakeIO);
  }

  public Command intake() {
    return intake.getIntakeCommand();
  }

  public Command getManualControlCommand(DoubleSupplier pivotSupplier) {
    return pivot.getManualOverrideCommand(pivotSupplier);
  }
}
