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

package frc.robot.subsystems.misc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.misc.interfaceLayers.LightsIO;
import frc.robot.subsystems.misc.interfaceLayers.LightsIOBlinkin.Colours;

public class LightSubsystem extends SubsystemBase {

  LightsIO light;

  public LightSubsystem(LightsIO io) {
    light = io;
  }

  public Command setColor(Colours color) {
    return runOnce(() -> light.setColor(color));
  }

  public Command setColorValue(int num) {
    return runOnce(() -> light.setColorValue(num));
  }

  public Command setColorNull() {
    return runOnce(() -> light.setColorValue(0));
  }
}
