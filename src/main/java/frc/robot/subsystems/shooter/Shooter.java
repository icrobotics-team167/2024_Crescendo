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

import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIOTimeOfFlight;

/** A class containing all the logic and commands to make the shooter mechanism work. */
public class Shooter {
  // private final FlywheelSubsystem flywheel;
  // private final PivotSubsystem pivot;
  private final NoteDetectorSubsystem noteDetector;

  public Shooter() {
    // TODO: Implement flywheel and pivot interfaces
    // flywheel = new FlywheelSubsystem(null);
    // pivot = new PivotSubsystem(null);
    noteDetector = new NoteDetectorSubsystem(new NoteDetectorIOTimeOfFlight());
  }
}
