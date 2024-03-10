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
import static edu.wpi.first.units.Units.Millimeters;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.util.CANConstants.Shooter;

public class NoteDetectorIOTimeOfFlight implements NoteDetectorIO {
  private final TimeOfFlight sensor;

  public NoteDetectorIOTimeOfFlight() {
    sensor = new TimeOfFlight(Shooter.NOTE_DETECTOR);
    sensor.setRangingMode(RangingMode.Short, 24);
  }

  LinearFilter rangeFilter = LinearFilter.movingAverage(5);

  @Override
  public void updateInputs(NoteDetectorIOInputs inputs) {
    if (sensor.isRangeValid()) {
      inputs.detectedDistance = Millimeters.of(rangeFilter.calculate(sensor.getRange()));
      inputs.hasNote = inputs.detectedDistance.lte(Inches.of(15.5)); // TODO: Tune
    } else {
      inputs.hasNote = false;
    }
  }
}
