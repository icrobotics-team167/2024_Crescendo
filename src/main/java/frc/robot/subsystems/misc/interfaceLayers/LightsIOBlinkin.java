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

public class LightsIOBlinkin implements LightsIO {
  PWM colourSpark;

  /** Cool colours. */
  public static enum Colours {
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
    BLACK(1995);

    int colourValue;
    /**
     * colours constructor
     *
     * @param colourValue colour value
     */
    private Colours(int colourValue) {
      this.colourValue = colourValue;
    }
  }

  /**
   * Lights constructor
   *
   * @param PWMID the PWMID of the blinky blinky light module (Spark motor)
   */
  public LightsIOBlinkin(int PWMID) {
    colourSpark = new PWM(PWMID);
  }

  /**
   * Set the colour for the spark motor bruv
   *
   * @param colour colours object bruv
   */
  public void setColour(Colours colour) {
    // set colour to set colourValue bruv
    colourSpark.setPulseTimeMicroseconds(colour.colourValue);
  }

  public void setColorValue(int colorValue) {
    colourSpark.setPulseTimeMicroseconds(colorValue);
  }

  public void setColorNull() {
    colourSpark.setPulseTimeMicroseconds(0);
  }
}
