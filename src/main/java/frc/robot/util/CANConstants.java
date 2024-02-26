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

public final class CANConstants {
  public static final String CANIVORE_NAME = "Croppenheimer";

  public final class Drivebase {
    public static final int GYRO = 14;
    public static final int FRONT_LEFT_DRIVE = 2;
    public static final int FRONT_LEFT_TURN = 3;
    public static final int FRONT_LEFT_ENCODER = 18;
    public static final int FRONT_RIGHT_DRIVE = 4;
    public static final int FRONT_RIGHT_TURN = 5;
    public static final int FRONT_RIGHT_ENCODER = 17;
    public static final int BACK_LEFT_DRIVE = 9;
    public static final int BACK_LEFT_TURN = 8;
    public static final int BACK_LEFT_ENCODER = 16;
    public static final int BACK_RIGHT_DRIVE = 7;
    public static final int BACK_RIGHT_TURN = 6;
    public static final int BACK_RIGHT_ENCODER = 15;
  }

  public final class Shooter {
    public static final int INTAKE = 10;
    public static final int FEEDER = 23;
    public static final int PIVOT_LEADER = 20;
    public static final int PIVOT_FOLLOWER = 19;
    public static final int TOP_FLYWHEEL = 22;
    public static final int BOTTOM_FLYWHEEL = 21;
    public static final int NOTE_DETECTOR = 32;
  }
}
