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

package frc.robot.util;

/** A class containing various math-related utilities. */
public class MathUtils {
  private MathUtils() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Applies an inner deadband, to account for stick drift, and an outer deadband, to account for
   * insufficient stick saturation. Values in between are linearly interpolated to provide a
   * continuous output.
   */
  public static double inOutDeadband(double value, double innerDeadband, double outerDeadband) {
    return inOutDeadband(value, innerDeadband, outerDeadband, 1);
  }

  /**
   * Applies an inner deadband, to account for stick drift, and an outer deadband, to account for
   * insufficient stick saturation. Values in between are interpolated to provide a continuous
   * output, with an exponent applied.
   */
  public static double inOutDeadband(
      double value, double innerDeadband, double outerDeadband, double outputExponent) {
    if (value < 0) {
      return -inOutDeadband(-value, innerDeadband, outerDeadband, outputExponent);
    }
    if (value < innerDeadband) {
      return 0;
    }
    if (value > outerDeadband) {
      return 1;
    }
    return Math.pow(
        (value - innerDeadband) / (1 - (innerDeadband + outerDeadband)), outputExponent);
  }
}
