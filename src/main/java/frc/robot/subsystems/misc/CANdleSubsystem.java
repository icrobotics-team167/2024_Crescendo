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

import com.ctre.phoenix.*;
import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CANConstants;

public class CANdleSubsystem extends SubsystemBase {

  private CANdle lights;

  public CANdleSubsystem(int ID) {
    this.lights = new CANdle(ID, CANConstants.CANIVORE_NAME);
    CANdleConfiguration config = new CANdleConfiguration();
    // config.stripType = LEDStripType.RGB;
    config.v5Enabled = true;
    lights.configAllSettings(config);
  }

  public Command setLight(Animation anim) {
    return runOnce(() -> lights.animate(anim, 1));
  }

  public Command clearLight(int i) {
    return runOnce(() -> lights.clearAnimation(1));
  }
}
