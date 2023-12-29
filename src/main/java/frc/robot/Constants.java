// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.helpers.MathUtils;
import frc.robot.helpers.Telemetry.Verbosity;

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
   * The telemetry verbosity level. Set to NONE during competitions to reduce data
   * being sent over the FMS and thus increasing connection stability.
   */
  public static final Verbosity TELEMETRY_VERBOSITY = Verbosity.HIGH;

  /**
   * How many seconds to lock drivebase motion after the end of the match.
   */
  public static final double END_OF_MATCH_LOCK = 10;

  /**
   * Field properties.
   */
  public static final class Field {
    /**
     * In some seasons, such as 2023, the field is assymetrical down the Y axis, so
     * trajectories/field positions need to be flipped when on the Red alliance.
     * Usually set to false since the 2023 field was weird.
     */
    public static final boolean ASSYMETRICAL_FIELD = true;

    /**
     * The length (long side) of the field, in meters.
     */
    public static final double FIELD_LENGTH = 16.5;

    /**
     * The width (short side) of the field, in meters.
     */
    public static final double FIELD_WIDTH = 8;
  }

  /**
   * Robot configuration and characteristics.
   */
  public static final class Robot {
    /**
     * Arm configuration and characteristics.
     */
    public static final class Arm {
      /**
       * Extension configuration and characteristics.
       */
      public static final class Extension {
        /**
         * The initial postion of the extension, in inches.
         */
        public static final double INITIAL_POSITION = 0;
        /**
         * The max extension of the arm, in inches.
         */
        public static final double EXTENSION_MAX = 20.0;
        /**
         * The min extension of the arm, in inches.
         */
        public static final double EXTENSION_MIN = INITIAL_POSITION;
        /**
         * The gear ratio from the extension motor to the pulley.
         */
        private static final double GEARBOX_RATIO = 81;
        /**
         * The distance the arm extends per rotation of the pulley, in inches.
         */
        private static final double PULLEY_RATIO = Math.PI * 2;
        /**
         * The distance the arm extends per rotation of the motor, in inches.
         */
        public static final double EXTENSION_GEAR_RATIO = PULLEY_RATIO / GEARBOX_RATIO;
      }

      /**
       * Pivot configuration and characteristics.
       */
      public static final class Pivot {
        /**
         * The initial position of pivot on robot boot, in degrees.
         */
        public static final double INITIAL_POSITION = 60.0;
        /**
         * The max position of the arm, in degrees.
         */
        public static final double PIVOT_MAX = 60.0;
        /**
         * The min position of the arm, in degrees.
         */
        public static final double PIVOT_MIN = -40.0;
        /**
         * The gear ratio of the pivot gear.
         */
        public static final double PIVOT_GEAR_RATIO = 400.0;
      }

      /**
       * Claw configuration and characteristics.
       */
      public static final class Claw {
        /**
         * The speed cap for the intake motor so it doesn't shred objects.
         */
        public static final double INTAKE_SPEED = 0.75;
        /**
         * The speed cap for the outtake motor so it doesn't yeet objects.
         */
        public static final double OUTTAKE_SPEED = 0.4;
      }

      /**
       * IDs for parts of the arm.
       */
      public static final class IDs {
        /**
         * The CAN ID of the right motor on the pivot mechanism.
         */
        public static final int PIVOT_LEADER = 10;
        /**
         * The CAN ID of the left motor on the pivot mechanism.
         */
        public static final int PIVOT_FOLLOWER = 11;
        /**
         * The CAN ID of the extension motor.
         */
        public static final int EXTENSION_MOTOR = 12;
        /**
         * The CAN ID of the claw motor.
         */
        public static final int CLAW_MOTOR = 13;
        /**
         * The DIO port of the minimum extension sensor.
         */
        public static final int MIN_EXTENSION_SENSOR = 0;
      }
    }

    /**
     * Swerve drivebase configuration and characteristics.
     */
    public static final class SwerveDrive {
      /**
       * The max translational velocity of the drivebase, in meters/s. Note that this
       * is not a module's absolute max speed.
       */
      public static final double MAX_TRANSLATIONAL_VEL = 2;
      /**
       * The max rotational velocity of the drivebase, in radians/s.
       */
      public static final double MAX_ROTATIONAL_VEL = 1.5 * Math.PI;
      /**
       * The max acceleration of the drivebase, in meters/s^2. Usually defined as the
       * CoF of the wheels * Gravity to prevent wheel slip.
       */
      public static final double MAX_ACCELERATION = Modules.WHEEL_COF * MathUtils.GRAVITY;
      /**
       * The time, in seconds, that the robot will take to go from 0 to full speed. Is
       * defined as (MAX_TRANSLATIONAL_VEL / MAX_ACCELERATION)
       */
      public static final double ZERO_TO_FULL_TIME = (MAX_TRANSLATIONAL_VEL / MAX_ACCELERATION);

      /**
       * Swerve module configuration and characteristics.
       */
      public static final class Modules {
        /**
         * Drive motor gear ratio, or how many times the drive motor will rotate before
         * 1 complete rotation of the wheel.
         */
        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
        /**
         * Diameter of the wheel, measured in meters.
         */
        public static final double WHEEL_DIAMETER = 4 * 0.0254; // 4 inches * 0.0254 inches per meter
        /**
         * Circumference of the wheel, measured in meters. Defined as WHEEL_DIAMATER
         * * Math.PI.
         */
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        /**
         * Coefficient of friction of the wheel.
         * TODO: Figure out if this is the right value or not.
         */
        public static final double WHEEL_COF = 0.77;

        public static final class ControlParams {
          public static final double DRIVE_P = 0.25;
          public static final double DRIVE_I = 0; // Don't use
          public static final double DRIVE_D = 0;
          public static final double TURN_P = 1.0 / 70;
          public static final double TURN_I = 0; // Don't use
          public static final double TURN_D = 1.0 / 160;
        }

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
           * The forward component of a module's distance to the robot center, in meters.
           */
          private static final double moduleForwardDistanceFromCenter = Units
              .inchesToMeters((robotLength - (2 * moduleCenterOfRotationDistanceFromEdge)) / 2.0);
          /**
           * The side component of a module's distance to the robot center, in meters.
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

          // Drivebase IDs
          /**
           * The turn encoder ID of the front left module. Could either be a CAN ID, a PWM
           * port number, or an Analog port nummber, depending on configuration.
           */
          public static final int FRONT_LEFT_ENCODER = 0;
          /**
           * The turn encoder ID of the front right module. Could either be a CAN ID, a
           * PWM port number, or an Analog port nummber, depending on configuration.
           */
          public static final int FRONT_RIGHT_ENCODER = 1;
          /**
           * The turn encoder ID of the back left module. Could either be a CAN ID, a PWM
           * port number, or an Analog port nummber, depending on configuration.
           */
          public static final int BACK_LEFT_ENCODER = 2;
          /**
           * The turn encoder ID of the back right module. Could either be a CAN ID, a PWM
           * port number, or an Analog port nummber, depending on configuration.
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
          public static final double FRONT_LEFT_OFFSET = 95;
          /**
           * The angle offset for the front right turn encoder.
           */
          public static final double FRONT_RIGHT_OFFSET = 53;
          /**
           * The angle offset for the back left turn encoder.
           */
          public static final double BACK_LEFT_OFFSET = 42;
          /**
           * The angle offset for the back right turn encoder.
           */
          public static final double BACK_RIGHT_OFFSET = 328;
        }
      }
    }

    /**
     * Autonomous period configurations.
     */
    public static final class Auto {
      /**
       * The path follower configuration for PathPlanner.
       */
      public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
          // TODO: Tune
          new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
          Robot.SwerveDrive.MAX_TRANSLATIONAL_VEL, // Max module speed, in m/s
          Robot.SwerveDrive.Modules.Positions.FRONT_LEFT_POS.getNorm(), // Drive base radius in meters. Distance from
                                                                        // robot center to furthest module.
          new ReplanningConfig() // Default path replanning config. See the API for the options here
      );
    }

    /**
     * Motor configuration and characteristics.
     */
    public static final class Motors {
      /**
       * Configuration and characteristics for the REV NEO 500.
       */
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
           * Primary current limit of the Rev NEO 500s, in amps. If the amperage exceeds
           * this amount, motor power will be reduced to compensate.
           */
          public static final int PRIMARY_CURRENT_LIMIT = 60;
          /**
           * Secondary current limit of the Rev NEO 500s, in amps. If the primary current
           * limit doesn't lower current draw enough and the amperage hits this value, the
           * motor will be temporarily shut down.
           */
          public static final int SECONDARY_CURRENT_LIMIT = 80;
        }

        /**
         * Max rotational velocity, measured in RPM.
         */
        public static final double MAX_RPM = 5700;
      }

      /**
       * Configuration and characteristics for the REV NEO 550.
       */
      public static final class Neo550 {
        /**
         * Motor power draw limits in order to prevent motor burnouts/other components
         * browning out.
         */
        public static final class CurrentLimits {
          /**
           * Nominal voltage of the Rev NEO 550s. If the voltage being sent to the NEOs
           * falls below this amount, motor power will be lowered to compensate.
           */
          public static final double NOMINAL_VOLTAGE = 12;
          /**
           * Primary current limit of the Rev NEO 550s, in amps. If the amperage exceeds
           * this amount, motor power will be reduced to compensate.
           */
          public static final int PRIMARY_CURRENT_LIMIT = 40;
          /**
           * Secondary current limit of the Rev NEO 550s, in amps. If the primary current
           * limit doesn't lower current draw enough and the amperage hits this value, the
           * motor will be temporarily shut down.
           */
          public static final int SECONDARY_CURRENT_LIMIT = 60;
        }

        /**
         * Max rotational velocity, measured in RPM.
         */
        public static final double MAX_RPM = 11000;
      }
    }
  }

  /**
   * Vision processing configurations.
   */
  public static final class Vision {
    /**
     * LimeLight configurations.
     */
    public static final class LimeLight {
      /**
       * NetworkTable camera name for the Limelight vision processor that's assigned
       * to detecting field AprilTags.
       */
      // TODO: Configure
      public static final String APRILTAG_DETECTOR = "LimeLight";
    }
  }

  /**
   * Driver configurations.
   */
  public static final class Driving {
    /**
     * Move speed multipliers for slow mode.
     */
    public static final double SLOWMODE_MULT = 0.25;

    /**
     * Controller configurations.
     */
    public static final class Controllers {
      /**
       * Controller IDs.
       */
      public static final class IDs {
        /**
         * Primary driver's left stick ID.
         */
        public static final int PRIMARY_LEFT = 0;
        /**
         * Primary driver's right stick ID.
         */
        public static final int PRIMARY_RIGHT = 1;
        /**
         * Secondary driver's left stick ID.
         */
        public static final int SECONDARY_LEFT = 2;
        /**
         * Secondary driver's left stick ID.
         */
        public static final int SECONDARY_RIGHT = 3;
      }

      /**
       * Controller deadbands.
       * TODO: Figure out if deadbands or deadzones feel better to use
       */
      public static final class Deadbands {
        /**
         * Primary driver's left stick deadband.
         */
        public static final double PRIMARY_LEFT = 0.05;
        /**
         * Primary driver's right stick deadband.
         */
        public static final double PRIMARY_RIGHT = 0.05;
        /**
         * Secondary driver's left stick deadband.
         */
        public static final double SECONDARY_LEFT = 0.03;
        /**
         * Secondary driver's right stick deadband.
         */
        public static final double SECONDARY_RIGHT = 0.03;
      }
    }

  }
}
