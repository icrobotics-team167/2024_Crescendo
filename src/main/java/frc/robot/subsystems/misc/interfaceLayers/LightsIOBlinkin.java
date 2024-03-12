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

import edu.wpi.first.wpilibj.PWM;
import frc.robot.subsystems.misc.LightSubsystem.LightState;
import frc.robot.util.CANConstants;

public class LightsIOBlinkin implements LightsIO {
  PWM colorBlinkin;
  int colorValue = 0;
  LightState commandedState = LightState.NO_NOTE;

  /** Cool colors. */
  private enum Colors {
    RAINBOW_PALETTE(1005),
    RAINBOW_GLITTER(1055),
    CONFETTI(1065),
    SHOT_RED(1075),
    SINELON_RAINBOX_PALETTE(1105),
    SINELON_LAVA_PALETTE(1135),
    BPM_OCEAN_PALETTE(1175),
    FIRE_LARGE(1215),
    // SOLID COLORS
    HOT_PINK(1785),
    DARK_RED(1795),
    RED(1805),
    RED_ORANGE(1815),
    ORANGE(1825),
    GOLD(1835),
    YELLOW(1845),
    LAWN_GREEN(1855),
    LIME(1865),
    DARK_GREEN(1875),
    GREEN(1885),
    BLUE_GREEN(1895),
    AQUA(1905),
    SKY_BLUE(1915),
    DARK_BLUE(1925),
    BLUE(1935),
    BLUE_VIOLET(1945),
    VIOLET(1955),
    WHITE(1965),
    GRAY(1975),
    DARK_GREY(1985),
    BLACK(1995),
    SECONDARY_BLINK(1675),
    GRADIENT(1705);

    int colorValue;
    /**
     * colors constructor
     *
     * @param colorValue color value
     */
    private Colors(int colorValue) {
      this.colorValue = colorValue;
    }
  }

  /**
   * Lights constructor
   *
   * @param PWMID the PWMID of the blinky blinky light module (Spark motor)
   */
  public LightsIOBlinkin() {
    // NOTE: 1705 is a good value
    colorBlinkin = new PWM(CANConstants.misc.LIGHT_PWM_ID_RIGHT);
  }

  @Override
  public void updateInputs(LightsIOInputs inputs) {
    inputs.state = commandedState;
  }

  @Override
  public void setColorFromState(LightState state) {
    commandedState = state;
    switch (state) {
      case NO_NOTE -> colorBlinkin.setPulseTimeMicroseconds(Colors.GREEN.colorValue);
      case INTAKING -> colorBlinkin.setPulseTimeMicroseconds(Colors.BLUE.colorValue);
      case HAS_NOTE -> colorBlinkin.setPulseTimeMicroseconds(Colors.GOLD.colorValue);
      case AIMING -> colorBlinkin.setPulseTimeMicroseconds(Colors.WHITE.colorValue);
      case AIM_OK -> colorBlinkin.setPulseTimeMicroseconds(Colors.FIRE_LARGE.colorValue);
      case SHOOTING -> colorBlinkin.setPulseTimeMicroseconds(Colors.GRADIENT.colorValue);
      default -> colorBlinkin.setPulseTimeMicroseconds(Colors.GREEN.colorValue);
    }
  }
}
