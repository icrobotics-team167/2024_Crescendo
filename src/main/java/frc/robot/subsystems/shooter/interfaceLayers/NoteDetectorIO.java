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

package frc.robot.subsystems.shooter.interfaceLayers;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface NoteDetectorIO {
  @AutoLog
  public class NoteDetectorIOInputs {
    /** If the detector at the shooter sees a note or not. */
    public boolean hasNoteInShooter = false;
    /** The distance from an object that the shooter detector has measured. */
    public Measure<Distance> shooterDetectedDistance = Inches.of(-1);
    /** If the detector at the shooter sees a note or not. */
    public boolean hasNoteInIntake = false;
    /** The distance from an object that the shooter detector has measured. */
    public Measure<Distance> intakeDetectedDistance = Inches.of(-1);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(NoteDetectorIOInputs inputs) {}
}
