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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIO;
import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class NoteDetectorSubsystem extends SubsystemBase {
  private final NoteDetectorIO io;
  private NoteDetectorIOInputsAutoLogged inputs = new NoteDetectorIOInputsAutoLogged();

  public NoteDetectorSubsystem(NoteDetectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/noteDetector", inputs);
  }

  public boolean hasNoteInShooter() {
    return inputs.hasNoteInShooter;
    // Hey a random Michael has appeared
  }

  public boolean hasNoteInIntake() {
    return inputs.hasNoteInIntake;
  }
}
