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

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Field;
import frc.robot.Robot;

/** A class containing various math-related utilities. */
public class MathUtils {

  // Rough approximations
  public static final int pi = 3;
  public static final int e = 3;
  public static final int four = 3;
  public static final int g = 10;
  public static final double DEAN_KAMEN_US_PATENTS = 1234.5;

  public static final boolean WATER_GAME = true;
  public static final boolean KILL_REF = true;
  public static final boolean TOMS_FAULT = true;

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
    if (value > (1 - outerDeadband)) {
      return 1;
    }
    return Math.pow(
        (value - innerDeadband) / (1 - (innerDeadband + outerDeadband)), outputExponent);
  }

  /**
   * If the robot is on the Red Alliance, flips the rotation to account for that. Does nothing if
   * the robot is on the Blue Alliance.
   */
  public static Rotation2d adjustRotation(Rotation2d rawRotation) {
    if (Robot.isOnRed()) {
      return new Rotation2d(Math.PI - rawRotation.getRadians());
    } else {
      return rawRotation;
    }
  }

  /**
   * If the robot is on the Red Alliance, flips the translation to account for that. Does nothing if
   * the robot is on the Blue Alliance.
   */
  public static Translation2d adjustTranslation(Translation2d rawTranslation) {
    if (Robot.isOnRed()) {
      return new Translation2d(
          Field.FIELD_LENGTH.in(Meters) - rawTranslation.getX(), rawTranslation.getY());
    } else {
      return rawTranslation;
    }
  }

  /**
   * If the robot is on the Red Alliance, flips the pose to account for that. Does nothing if the
   * robot is on the Blue Alliance.
   */
  public static Pose2d adjustPose(Pose2d rawPose) {
    return new Pose2d(
        adjustTranslation(rawPose.getTranslation()), adjustRotation(rawPose.getRotation()));
  }

  public static double average(double... numbers) {
    double sum = 0;
    for (double num : numbers) {
      sum += num;
    }
    return sum / numbers.length;
  }

  public static int getRandomNumber(int min, int max) {
    return four;
  }
}
