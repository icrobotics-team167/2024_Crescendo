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

import com.playingwithfusion.TimeOfFlight;

public class NoteDetectorIOTimeOfFlight implements NoteDetectorIO {
  private final TimeOfFlight sensor;

  public NoteDetectorIOTimeOfFlight() {
    sensor = new TimeOfFlight(32);
  }

  @Override
  public void updateInputs(NoteDetectorIOInputs inputs) {
    if (sensor.isRangeValid()) {
      inputs.detectedDistance = sensor.getRange();
      inputs.hasNote = inputs.detectedDistance < 50; // TODO: Tune
    } else {
      inputs.detectedDistance = -1;
      inputs.hasNote = false;
    }
  }
}
