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

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static boolean IS_ON_RED =
      DriverStation.getAlliance().isPresent()
          ? DriverStation.getAlliance().get() == Alliance.Red
          : false;

  public static final class Field {
    public static final Measure<Distance> FIELD_LENGTH = Meters.of(16.54);
    public static final Measure<Distance> FIELD_WIDTH = Meters.of(8.21);
  }

  public static final class Driving {
    public static final boolean SLOWMODE_DEFAULT = true;
    public static final double SLOWMODE_MULTIPLIER = 0.2;
    public static final double PRIMARY_DRIVER_EXPONENT = 1.5;
    public static final double SECONDARY_DRIVER_EXPONENT = 1;

    public static final class Deadbands {
      public static final double PRIMARY_LEFT_INNER = 0.05;
      public static final double PRIMARY_RIGHT_INNER = 0.05;
      public static final double SECONDARY_LEFT_INNER = 0.05;
      public static final double SECONDARY_RIGHT_INNER = 0.05;
      public static final double PRIMARY_LEFT_OUTER = 0.01;
      public static final double PRIMARY_RIGHT_OUTER = 0.01;
      public static final double SECONDARY_LEFT_OUTER = 0.01;
      public static final double SECONDARY_RIGHT_OUTER = 0.01;
    }
  }
}
