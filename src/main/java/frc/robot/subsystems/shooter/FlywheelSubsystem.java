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

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.interfaceLayers.FlywheelIO;
import frc.robot.subsystems.shooter.interfaceLayers.FlywheelIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class FlywheelSubsystem extends SubsystemBase {
  private final FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/flywheel", inputs);
  }

  /**
   * Gets the command to spin up the flywheel to shoot into the speaker. Stops spinning when the
   * command ends.
   */
  public Command getSpeakerShotCommand() {
    return run(io::runSpeaker).finallyDo(io::stop);
  }

  /**
   * Gets the command to spin up the flywheel to shoot into the amp. Stops spinning when the command
   * ends.
   */
  public Command getAmpShotCommand() {
    return run(io::runAmp).finallyDo(io::stop);
  }

  public boolean isUpToSpeed() {
    System.out.println("Flywheel setpoint: " + inputs.velocitySetpointRPM);
    if (inputs.velocitySetpointRPM == 0) {
      return false;
    }
    System.out.println("Is up to speed?: " + String.valueOf(inputs.bottomVelocity.in(RPM) >= inputs.velocitySetpointRPM));
    return inputs.bottomVelocity.in(RPM) >= inputs.velocitySetpointRPM;
  }
}
