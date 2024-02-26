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

package frc.robot.subsystems.misc.interfaceLayers;

import frc.robot.subsystems.misc.interfaceLayers.LightsIOBlinkin.Colours;

public interface LightsIO {

  public class LightsIOInputs {
    // TODO: Implement
  }

  public default void updateInputs(LightsIOInputs inputs) {}

  public default void setColor(Colours color) {}

  public default void setColorValue(int num) {}

  public default void setColorNull() {}
}
