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
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import frc.robot.subsystems.misc.LightSubsystem.LightState;

public class LightsIOCANdle implements LightsIO {
  CANdle candle;

  public LightsIOCANdle() {
    candle = new CANdle(40, "Croppenheimer");

    CANdleConfiguration config = new CANdleConfiguration();
    // config.v5Enabled = true;
    config.statusLedOffWhenActive = true;
    config.disableWhenLOS = false;
    config.stripType = LEDStripType.RGB;
    candle.configAllSettings(config);
  }

  private static final int LIGHT_NUM = 42;

  @Override
  public void setColorFromState(LightState state) {
    Animation animation;
    switch (state) {
      case AIMING -> {
        animation = new RainbowAnimation(1, 1, LIGHT_NUM, false, 0);
      }
      case AIM_OK -> {
        animation = new ColorFlowAnimation(0, 255, 0, 0, 1, LIGHT_NUM, Direction.Forward, 0);
      }
      case HAS_NOTE -> {
        animation = new TwinkleAnimation(0, 255, 0, 0, 1, LIGHT_NUM, TwinklePercent.Percent100, 0);
      }
      case INDEXING_NOTE -> {
        animation = new StrobeAnimation(0, 255, 0, 0, 1, LIGHT_NUM, 0);
      }
      case INTAKING -> {
        animation = new RgbFadeAnimation(1, 1, LIGHT_NUM, 0);
      }
      case NO_NOTE -> {
        animation = new FireAnimation(1, .7, LIGHT_NUM, .8, .75);
      }
      case SHOOTING -> {
        animation = new LarsonAnimation(0, 255, 0, 0, 1, LIGHT_NUM, BounceMode.Front, 2, 0);
      }
      default -> {
        animation = null;
      }
    }
    candle.animate(animation);
  }

  public void setLEDTest() {
    candle.setLEDs(255,0,0,0,9,3);
  }
}
