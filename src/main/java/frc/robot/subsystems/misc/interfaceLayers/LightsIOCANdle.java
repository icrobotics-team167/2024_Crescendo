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

import com.ctre.phoenix.led.*;
import frc.robot.subsystems.misc.LightSubsystem.LightState;

public class LightsIOCANdle implements LightsIO {
  CANdle candle;

  public LightsIOCANdle() {
    candle = new CANdle(26, "Croppenheimer");

    CANdleConfiguration config = new CANdleConfiguration();
    candle.configAllSettings(config);
  }

  @Override
  public void setColorFromState(LightState state) {
    switch (state) {
      case AIMING -> {
        candle.animate(new RainbowAnimation());
      }
      case AIM_OK -> {
        candle.animate(new ColorFlowAnimation(0, 255, 0));
      }
      case HAS_NOTE -> {
        candle.animate(new TwinkleAnimation(0, 255, 0));
      }
      case INDEXING_NOTE -> {
        candle.animate(new StrobeAnimation(0, 255, 0));
      }
      case INTAKING -> {
        candle.animate(new FireAnimation());
      }
      case NO_NOTE -> {
        candle.animate(new RgbFadeAnimation());
      }
      case SHOOTING -> {
        candle.animate(new LarsonAnimation(0, 255, 0));
      }
    }
  }
}
