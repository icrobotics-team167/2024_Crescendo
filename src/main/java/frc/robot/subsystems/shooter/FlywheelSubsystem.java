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
import org.littletonrobotics.junction.AutoLogOutput;
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

  public Command getSourceIntakeCommand() {
    return run(io::runSourceIntake).finallyDo(io::stop);
  }

  @AutoLogOutput
  public boolean isUpToSpeed() {
    if (inputs.topVelocitySetpoint.in(RPM) == 0 || inputs.bottomVelocitySetpoint.in(RPM) == 0) {
      return false;
    }

    return atSetpoint(inputs.topVelocity.in(RPM), inputs.topVelocitySetpoint.in(RPM))
        && atSetpoint(inputs.bottomVelocity.in(RPM), inputs.bottomVelocitySetpoint.in(RPM))
        && atSetpoint(inputs.guideVelocity.in(RPM), inputs.guideWheelSetpoint.in(RPM));
  }

  private boolean atSetpoint(double velocity, double setpoint) {
    if (setpoint < 0) {
      return atSetpoint(-velocity, -setpoint);
    }
    return velocity >= setpoint;
  }
}
