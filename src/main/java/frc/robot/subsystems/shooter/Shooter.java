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

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A class containing all the logic and commands to make the shooter mechanism work. */
public class Shooter extends SubsystemBase {
  // private final FlywheelSubsystem flywheel;
  // private final PivotSubsystem pivot;
  private static CANSparkMax intake;

  public Shooter() {
    // TODO: Implement flywheel and pivot interfaces
    // flywheel = new FlywheelSubsystem(null);
    // pivot = new PivotSubsystem(null);
    intake = new CANSparkMax(10, MotorType.kBrushless);
  }

  public Command run() {
    System.out.println("kill me");
    return run(() -> intake.set(.5));
  }

  public Command stop() {
    return run(() -> intake.stopMotor());
  }
}
