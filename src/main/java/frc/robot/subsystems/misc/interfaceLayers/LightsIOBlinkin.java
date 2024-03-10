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
import frc.robot.util.CANConstants;

public class LightsIOBlinkin implements LightsIO {
  PWM colorSparkRight;
  int colorValue = 0;

  /** Cool colors. */
  public static enum Colors {
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
    colorSparkRight = new PWM(CANConstants.misc.LIGHT_PWM_ID_RIGHT);
  }

  @Override
  public void updateInputs(LightsIOInputs inputs) {
    inputs.colorValue = colorValue;
  }

  /**
   * Set the color for the spark motor bruv
   *
   * @param color colors object bruv
   */
  @Override
  public void setColor(Colors color) {
    // set color to set colorValue bruv
    setColorValue(color.colorValue);
  }

  public void setColorValue(int colorValue) {
    this.colorValue = colorValue;
    colorSparkRight.setPulseTimeMicroseconds(colorValue);
  }

  public void setColorNull() {
    // Black is essentially just off, cuz black isn't real. I don't understand why we can't get
    // diversity awards
    colorSparkRight.setPulseTimeMicroseconds(1995);
  }
}
