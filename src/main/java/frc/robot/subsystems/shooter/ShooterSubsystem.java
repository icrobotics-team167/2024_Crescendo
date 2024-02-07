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

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIO;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIOInputsAutoLogged;
import frc.robot.subsystems.shooter.interfaceLayers.FlywheelIO;
import frc.robot.subsystems.shooter.interfaceLayers.ShooterIOInputsAutoLogged;

public class ShooterSubsystem extends SubsystemBase {
  private final PivotIO pivot;
  private PivotIOInputsAutoLogged pivotInputs;

  private final FlywheelIO flywheel;
  private ShooterIOInputsAutoLogged shooterInputs;

  public ShooterSubsystem(PivotIO pivot, FlywheelIO flywheel) {
    this.pivot = pivot;
    this.flywheel = flywheel;
  }

  @Override
  public void periodic() {
    pivot.updateInputs(pivotInputs);
    Logger.processInputs("Shooter/pivot", pivotInputs);
    flywheel.updateInputs(shooterInputs);
    Logger.processInputs("Shooter/flywheel", shooterInputs);
  }
}
