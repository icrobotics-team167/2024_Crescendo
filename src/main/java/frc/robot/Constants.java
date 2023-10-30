// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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
  /**
   * Robot configuration and characteristics.
   */
  public static final class Robot {
    /**
     * Swerve drivebase configuration and characteristics.
     */
    public static final class SwerveDrive {
      /**
       * The max translational velocity of the drivebase, in meters/s.
       */
      public static final double MAX_TRANSLATIONAL_VEL = 3.5;
      /**
       * The max rotational velocity of the drivebase, in radians/s.
       */
      public static final double MAX_ROTATIONAL_VEL = 1.5 * Math.PI;
      /**
       * The multiplier for slow mode.
       */
      public static final double SLOWMODE_MULT = 0.4;

      /**
       * Swerve module configuration and characteristics.
       */
      public static final class Modules {
        /**
         * Drive motor gear ratio, or how many times the drive motor will rotate before
         * 1 complete rotation of the wheel.
         */
        public static final double GEAR_RATIO = 6.75;
        /**
         * Diameter of the wheel, measured in meters.
         */
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
        /**
         * Circumference of the wheel, measured in meters. Defined as WHEEL_DIAMATER
         * * Math.PI.
         */
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        /**
         * Coefficient of friction of the wheel.
         */
        public static final double WHEEL_COF = 1.19;

        /**
         * Positions of the modules, relative to the center of the robot, measured in
         * meters.
         */
        public static final class Positions {
          /**
           * Length of the robot, in inches. Includes the bumper.
           */
          private static final double robotLength = 35;
          /**
           * Width of the robot, in inches. Includes the bumper.
           */
          private static final double robotWidth = 34;
          /**
           * Distance of the center of rotation from the edge of the bumper.
           */
          // 2.625 inches from the edge of the module + 2.5 inches of bumper
          private static final double moduleCenterOfRotationDistanceFromEdge = 2.625 + 2.5;
          /**
           * The forward component of the robot's distance to the center, in meters.
           */
          private static final double moduleForwardDistanceFromCenter = Units
              .inchesToMeters((robotLength - (2 * moduleCenterOfRotationDistanceFromEdge)) / 2.0);
          /**
           * The side component of the robot's distance to the center, in meters.
           */
          private static final double moduleSideDistanceFromCenter = Units
              .inchesToMeters((robotWidth - (2 * moduleCenterOfRotationDistanceFromEdge)) / 2.0);

          // TODO: Figure out which way positive x and positive y goes
          /**
           * A Translation2d object, representing the position of the front left swerve
           * module relative to the center of the robot.
           */
          public static final Translation2d FRONT_LEFT_POS = new Translation2d(-moduleForwardDistanceFromCenter,
              -moduleSideDistanceFromCenter);
          /**
           * A Translation2d object, representing the position of the front right swerve
           * module relative to the center of the robot.
           */
          public static final Translation2d FRONT_RIGHT_POS = new Translation2d(
              -moduleForwardDistanceFromCenter, moduleSideDistanceFromCenter);
          /**
           * A Translation2d object, representing the position of the back left swerve
           * module relative to the center of the robot.
           */
          public static final Translation2d BACK_LEFT_POS = new Translation2d(moduleForwardDistanceFromCenter,
              -moduleSideDistanceFromCenter);
          /**
           * A Translation2d object, represnting the position of the back right swerve
           * module relative to the center of the robot.
           */
          public static final Translation2d BACK_RIGHT_POS = new Translation2d(moduleForwardDistanceFromCenter,
              moduleSideDistanceFromCenter);
        }

        /**
         * CAN IDs/RoboRIO port numbers for various parts of the modules.
         */
        public static final class IDs {
          // Drivebase CAN bus Addresses
          /**
           * The drive motor CAN ID for the front left module.
           */
          public static final int FRONT_LEFT_DRIVE = 2;
          /**
           * The turn motor CAN ID for the front left module.
           */
          public static final int FRONT_LEFT_TURN = 3;
          /**
           * The drive motor CAN ID for the front right module.
           */
          public static final int FRONT_RIGHT_DRIVE = 4;
          /**
           * The turn motor CAN ID for the front right module.
           */
          public static final int FRONT_RIGHT_TURN = 5;
          /**
           * The drive motor CAN ID for the back left module.
           */
          public static final int BACK_LEFT_DRIVE = 9;
          /**
           * The turn motor CAN ID for the back left module.
           */
          public static final int BACK_LEFT_TURN = 8;
          /**
           * The drive motor CAN ID for the back right module.
           */
          public static final int BACK_RIGHT_DRIVE = 6;
          /**
           * The turn motor CAN ID for the back right module.
           */
          public static final int BACK_RIGHT_TURN = 7;

          // Drivebase Analog Encoder Ports
          /**
           * The turn encoder analog port number of the front left module.
           */
          public static final int FRONT_LEFT_ENCODER = 0;
          /**
           * The turn encoder analog port number of the front right module.
           */
          public static final int FRONT_RIGHT_ENCODER = 1;
          /**
           * The turn encoder analog port number of the back left module.
           */
          public static final int BACK_LEFT_ENCODER = 2;
          /**
           * The turn encoder analog port number of the back right module.
           */
          public static final int BACK_RIGHT_ENCODER = 3;
        }

        /**
         * The angles at which the encoders should be offset by, in degrees.
         */
        public static final class EncoderOffsets {
          /**
           * The angle offset for the front left turn encoder.
           */
          public static final double FRONT_LEFT_OFFSET = -95;
          /**
           * The angle offset for the front right turn encoder.
           */
          public static final double FRONT_RIGHT_OFFSET = -53;
          /**
           * The angle offset for the back left turn encoder.
           */
          public static final double BACK_LEFT_OFFSET = -42;
          /**
           * The angle offset for the back right turn encoder.
           */
          public static final double BACK_RIGHT_OFFSET = 32;
        }
      }
    }

    /**
     * Motor configuration and characteristics.
     */
    public static final class Motors {
      public static final class Neo500 {
        /**
         * Motor power draw limits in order to prevent motor burnouts/other components
         * browning out.
         */
        public static final class CurrentLimits {
          /**
           * Nominal voltage of the Rev NEO 500s. If the voltage being sent to the NEOs
           * falls below this amount, motor power will be lowered to compensate.
           */
          public static final double NOMINAL_VOLTAGE = 12;
          /**
           * Primary current limit of the Rev NEO 500s. If the amperage exceeds this
           * amount, motor power will be reduced to compensate.
           */
          public static final int PRIMARY_CURRENT_LIMIT = 80;
          /**
           * Secondary current limit of the Rev NEO 500s. If the primary current limit
           * doesn't lower current draw enough and the amperage hits this value, the motor
           * will be temporarily shut down completely.
           */
          public static final int SECONDARY_CURRENT_LIMIT = 100;
        }

        /**
         * Max rotational velocity, measured in RPM.
         */
        public static final double MAX_RPM = 5700;
      }
    }
  }
}
