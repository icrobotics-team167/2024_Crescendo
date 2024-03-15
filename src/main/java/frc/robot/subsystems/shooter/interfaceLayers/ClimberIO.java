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

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public Rotation2d leftAngle = new Rotation2d();
    public Rotation2d rightAngle = new Rotation2d();
    public Rotation2d angleSetpoint = new Rotation2d();
    public Measure<Velocity<Angle>> angularVelocity = RadiansPerSecond.of(0);
    public Measure<Voltage> leftAppliedVoltage = Volts.of(0);
    public Measure<Current> leftAppliedCurrent = Amps.of(0);
    public Measure<Voltage> rightAppliedVoltage = Volts.of(0);
    public Measure<Current> rightAppliedCurrent = Amps.of(0);
    public Measure<Velocity<Angle>> leftVelocity = DegreesPerSecond.of(0);
    public Measure<Velocity<Angle>> rightVelocity = DegreesPerSecond.of(0);
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void manualControl(double control) {}

  public default void stop() {}
}
