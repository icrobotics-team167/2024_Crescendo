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

import frc.robot.subsystems.misc.LightSubsystem.LightState;
import org.littletonrobotics.junction.AutoLog;

public interface LightsIO {
  @AutoLog
  public class LightsIOInputs {
    public LightState state = LightState.NO_NOTE;
  }

  public default void updateInputs(LightsIOInputs inputs) {}

  public default void setColorFromState(LightState state) {}

  public default void setColorNull() {}

  public default void setLEDTest() {}
}
