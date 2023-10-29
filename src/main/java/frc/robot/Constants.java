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
  /**
   * Robot configuration.
   */
  public final class Robot {
    /**
     * Motor configuration and characteristics.
     */
    public final class Motors {
      /**
       * Motor power draw limits in order to prevent motor burnouts/other components
       * browning out.
       */
      public final class CurrentLimits {
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
