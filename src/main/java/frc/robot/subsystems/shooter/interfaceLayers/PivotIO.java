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

package frc.robot.subsystems.shooter.interfaceLayers;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  public static final double MIN_ANGLE = 10;
  public static final double MAX_ANGLE = 90;

  @AutoLog
  public class PivotIOInputs {
    /** The angle of the pivot mechanism. 0 degrees is parallel to the ground. */
    public Rotation2d angle = new Rotation2d();
    /** If the pivot mechanism has reached its absolute max pivot angle. */
    public boolean isTooFarUp = false;
    /** If the pivot mechanism has reached its absolute min pivot angle. */
    public boolean isTooFarDown = false;
    /** The target angle of the pivot, if under closed loop control. */
    public Rotation2d angleSetpoint = new Rotation2d();
    /** The velocity of the leader motor. */
    public Measure<Velocity<Angle>> leaderVelocity = DegreesPerSecond.of(0);
    /** The velocity of the follower motor. */
    public Measure<Velocity<Angle>> followerVelocity = DegreesPerSecond.of(0);
    /** The target velocity of the motors. */
    public Measure<Velocity<Angle>> velocitySetpoint = DegreesPerSecond.of(0);
    /** The voltage applied to the motor by the motor controller. */
    public Measure<Voltage> leaderAppliedVoltage = Volts.of(0);
    /** The current applied to the motor by the motor controller. */
    public Measure<Current> leaderAppliedCurrent = Amps.of(0);
    /** The voltage applied to the motor by the motor controller. */
    public Measure<Voltage> followerAppliedVoltage = Volts.of(0);
    /** The current applied to the motor by the motor controller. */
    public Measure<Current> followerAppliedCurrent = Amps.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Sets the target angle for the pivot mechanism. 0 degrees is parallel to the ground. */
  public default void setTargetAngle(Rotation2d angle) {}

  /** Sets the closed-loop velocity control for the mechanism. */
  public default void setVelocityControl(Measure<Velocity<Angle>> velocity) {}

  /** Sets the raw open-loop velocity control for the motors. */
  public default void setRawControl(Measure<Voltage> voltage) {}

  /** Stops the pivot mechanism and keeps it at its current angle. */
  public default void stop() {}
}
