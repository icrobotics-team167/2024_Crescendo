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

package frc.robot.util.MotorUtils;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import java.util.Set;

// Modified version of code written by Team 1155 Sciborgs
// https://github.com/SciBorgs/Hydrogen/blob/main/src/main/java/org/sciborgs1155/lib/SparkUtils.java
/** Utility class for configuration of Spark motor controllers */
public class SparkUtils {

  public static final int FRAME_STRATEGY_DISABLED = 65535;
  public static final int FRAME_STRATEGY_SLOW = 400;
  public static final int FRAME_STRATEGY_FAST = 15;

  public static final Angle ANGLE_UNIT = Units.Rotations;
  public static final Time TIME_UNIT = Units.Minutes;
  public static final Angle THROUGHBORE_PPR =
      Units.derive(Rotations).splitInto(2048).named("Pulses Per Revolution").symbol("PPR").make();

  /** Represents a type of sensor that can be plugged into the spark */
  public static enum Sensor {
    INTEGRATED,
    ANALOG,
    QUADRATURE,
    DUTY_CYCLE;
  }

  /** Represents a type of data that can be sent from the spark */
  public static enum Data {
    POSITION,
    VELOCITY,
    CURRENT,
    VOLTAGE,
    TEMPERATURE;
  }

  /**
   * Configures CAN frames periods on a spark to send only specified data at high rates.
   *
   * @param spark The Spark MAX or Spark FLEX to configure.
   * @param data The data that the spark needs to send to the RIO.
   * @param sensors The sensors that provide data for the spark needs to send to the RIO.
   * @param withFollower Whether this spark has a following motor via {@link
   *     CANSparkBase#follow(CANSparkBase)}.
   * @see Sensor
   * @see Data
   * @see https://docs.revrobotics.com/brushless/spark-max/control-interfaces
   */
  public static void configureFrameStrategy(
      CANSparkBase spark, Set<Data> data, Set<Sensor> sensors, boolean withFollower) {
    int status0 = 10; // output, faults
    int status1 = FRAME_STRATEGY_DISABLED;
    // integrated velocity, temperature, input voltage, current | default 20
    int status2 = FRAME_STRATEGY_DISABLED; // integrated position | default 20
    int status3 = FRAME_STRATEGY_DISABLED; // analog encoder | default 50
    int status4 = FRAME_STRATEGY_DISABLED; // alternate quadrature encoder | default 20
    int status5 = FRAME_STRATEGY_DISABLED; // duty cycle position | default 200
    int status6 = FRAME_STRATEGY_DISABLED; // duty cycle velocity | default 200
    int status7 = FRAME_STRATEGY_DISABLED; // Iaccum | default 250

    if (sensors.contains(Sensor.INTEGRATED) && data.contains(Data.VELOCITY)
        || data.contains(Data.VOLTAGE)
        || data.contains(Data.CURRENT)
        || data.contains(Data.TEMPERATURE)) {
      status1 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.INTEGRATED) && data.contains(Data.POSITION)) {
      status2 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.ANALOG)
        && (data.contains(Data.VOLTAGE)
            || data.contains(Data.VELOCITY)
            || data.contains(Data.POSITION))) {
      status3 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.QUADRATURE)
        && (data.contains(Data.VELOCITY) || data.contains(Data.POSITION))) {
      status4 = FRAME_STRATEGY_FAST;
    }

    if (sensors.contains(Sensor.DUTY_CYCLE)) {
      if (data.contains(Data.POSITION)) {
        status5 = FRAME_STRATEGY_FAST;
      }
      if (data.contains(Data.VELOCITY)) {
        status6 = FRAME_STRATEGY_FAST;
      }
    }

    if (!withFollower) {
      status0 = FRAME_STRATEGY_SLOW;
    }

    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, status0);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, status3);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, status4);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, status5);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, status6);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus7, status7);
  }

  /**
   * Configures a follower spark to send nothing except output and faults. This means most data will
   * not be accessible.
   *
   * @param spark The follower spark.
   */
  public static void configureFollowerFrameStrategy(CANSparkBase spark) {
    configureFrameStrategy(spark, Set.of(), Set.of(), false);
  }

  /**
   * Configures a CANSpark motor.
   *
   * @param inverted The state of inversion. True if inverted.
   * @param idleMode Idle mode setting (either kCoast or kBrake).
   * @param limit current limit in Amps.
   * @param spark The motor object. This is either a CANSparkMax object or a CANSparkFlex object.
   */
  public static void configureSettings(
      boolean inverted, IdleMode idleMode, Measure<Current> limit, CANSparkBase spark) {
    spark.restoreFactoryDefaults();
    spark.setInverted(inverted);
    spark.setIdleMode(idleMode);
    spark.setSmartCurrentLimit((int) limit.in(Amps));
    spark.enableVoltageCompensation(12);
  }

  /**
   * Configures multiple CANSpark motors.
   *
   * @param inverted The state of inversion. True if inverted.
   * @param idleMode Idle mode setting (either kCoast or kBrake).
   * @param limit current limit in Amps.
   * @param sparks The spark objects. These can either be a CANSparkMax object or a CANSparkFlex
   *     object.
   */
  public static void configureSettings(
      boolean inverted, IdleMode idleMode, Measure<Current> limit, CANSparkBase... sparks) {
    for (var spark : sparks) {
      configureSettings(inverted, idleMode, limit, spark);
    }
  }
}
