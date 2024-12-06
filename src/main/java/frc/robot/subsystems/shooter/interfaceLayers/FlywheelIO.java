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

public interface FlywheelIO {
  @AutoLog
  public class FlywheelIOInputs {
    public Measure<Velocity<Angle>> topVelocitySetpoint = RPM.of(0);
    public Measure<Velocity<Angle>> bottomVelocitySetpoint = RPM.of(0);
    public Measure<Velocity<Angle>> guideWheelSetpoint = RPM.of(0);
    /** The position of the top shooter flywheel. */
    public Measure<Angle> topPosition = Rotations.of(0);
    /** The velocity of the top shooter flywheel. */
    public Measure<Velocity<Angle>> topVelocity = RotationsPerSecond.of(0);
    /** The total output applied to the top motor by the closed loop control. */
    public double topAppliedOutput = 0;
    /** The voltage applied to the top motor by the motor controller. */
    public Measure<Voltage> topAppliedVoltage = Volts.of(0);
    /** The current applied to the top motor by the motor controller. */
    public Measure<Current> topAppliedCurrent = Amps.of(0);
    /** The position of the bottom shooter flywheel. */
    public Measure<Angle> bottomPosition = Rotations.of(0);
    /** The velocity of the bottom shooter flywheel. */
    public Measure<Velocity<Angle>> bottomVelocity = RotationsPerSecond.of(0);
    /** The total output applied to the bottom motor by the closed loop control. */
    public double bottomAppliedOutput = 0;
    /** The voltage applied to the bottom motor by the motor controller. */
    public Measure<Voltage> bottomAppliedVoltage = Volts.of(0);
    /** The current applied to the bottom motor by the motor controller. */
    public Measure<Current> bottomAppliedCurrent = Amps.of(0);
    /** The position of the shooter guide wheels. */
    public Measure<Angle> guidePosition = Rotations.of(0);
    /** The velocity of the shooter guide wheels. */
    public Measure<Velocity<Angle>> guideVelocity = RotationsPerSecond.of(0);
    /** The total output applied to the guide wheel motor by the closed loop control. */
    public double guideAppliedOutput = 0;
    /** The voltage applied to the guide wheel motor by the motor controller. */
    public Measure<Voltage> guideAppliedVoltage = Volts.of(0);
    /** The current applied to the guide wheel motor by the motor controller. */
    public Measure<Current> guideAppliedCurrent = Amps.of(0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Spins the shooter flywheels up to shoot into the speaker. */
  public default void runSpeaker() {}

  /** Spins the shooter flywheels up to shoot into the amp. */
  public default void runAmp() {}

  /** Spins the shooter flywheels up to intake from source. */
  public default void runSourceIntake() {}

  /** Stops the shooter flywheel. */
  public default void stop() {}
}
