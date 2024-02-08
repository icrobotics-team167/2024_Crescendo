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
  @AutoLog
  public class PivotIOInputs {
    /** The angle of the pivot mechanism. 0 degrees is parallel to the ground. */
    public Rotation2d angle = new Rotation2d();
    /** If the pivot mechanism has reached its absolute max pivot angle. */
    public boolean isTooFarUp = false;
    /** If the pivot mechanism has reached its absolute min pivot angle. */
    public boolean isTooFarDown = false;
    /** The velocity of the pivot mechanism. */
    public Measure<Velocity<Angle>> velocity = DegreesPerSecond.of(0);
    /** The total output applied to the motor by the closed loop control. */
    public double appliedOutput = 0;
    /** The voltage applied to the motor by the motor controller. */
    public Measure<Voltage> appliedVoltage = Volts.of(0);
    /** The current applied to the motor by the motor controller. */
    public Measure<Current> appliedCurrent = Amps.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Sets the target angle for the pivot mechanism. 0 degrees is parallel to the ground. */
  public default void setTargetAngle(Rotation2d angle) {}

  /** Sets the open-loop control for the pivot mechanism. */
  public default void setPivotControl(Measure<Voltage> rawVolts) {}

  /** Stops the pivot mechanism and keeps it at its current angle. */
  public default void stop() {}
}
