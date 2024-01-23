// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class Driving {
    public static final boolean SLOWMODE_DEFAULT = true;
    public static final double SLOWMODE_MULTIPLIER = 0.2;

    public static final class Deadbands {
      public static final double PRIMARY_LEFT = 0.05;
      public static final double PRIMARY_RIGHT = 0.05;
      public static final double SECONDARY_LEFT = 0.05;
      public static final double SECONDARY_RIGHT = 0.05;
    }
  }
}
