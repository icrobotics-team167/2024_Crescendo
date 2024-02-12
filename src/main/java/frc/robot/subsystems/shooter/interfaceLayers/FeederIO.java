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

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public class FeederIOInputs {
    /** The position of the shooter flywheel. */
    public Measure<Angle> position = Rotations.of(0);
    /** The velocity of the shooter flywheel. */
    public Measure<Velocity<Angle>> velocity = RotationsPerSecond.of(0);
    /** The total output applied to the motor by the closed loop control. */
    public double appliedOutput = 0;
    /** The voltage applied to the motor by the motor controller. */
    public Measure<Voltage> appliedVoltage = Volts.of(0);
    /** The current applied to the motor by the motor controller. */
    public Measure<Current> appliedCurrent = Amps.of(0);
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void run() {}

  public default void stop() {}
}
