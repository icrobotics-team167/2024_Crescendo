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

package frc.robot.subsystems.misc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.misc.interfaceLayers.LightsIO;
import frc.robot.subsystems.misc.interfaceLayers.LightsIOInputsAutoLogged;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class LightSubsystem extends SubsystemBase {
  LightsIO io;
  LightsIOInputsAutoLogged inputs = new LightsIOInputsAutoLogged();

  public LightSubsystem(LightsIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.setColorFromState(currentState);
    io.updateInputs(inputs);
    Logger.processInputs("Misc/lights", inputs);
  }

  public enum LightState {
    NO_NOTE,
    INTAKING,
    HAS_NOTE,
    AIMING,
    AIM_OK,
    SHOOTING;
  }

  private LightState currentState = LightState.NO_NOTE;

  public Command setState(
      BooleanSupplier hasNote,
      BooleanSupplier isIntaking,
      BooleanSupplier isAiming,
      BooleanSupplier isAimOK,
      BooleanSupplier isShooting) {
    return run(
        () -> {
          switch (currentState) {
            case NO_NOTE -> {
              if (isIntaking.getAsBoolean()) {
                // If intaking, switch to the "intaking" state
                currentState = LightState.INTAKING;
              }
            }
            case INTAKING -> {
              if (!isIntaking.getAsBoolean()) {
                // If the intake gets stopped
                if (hasNote.getAsBoolean()) {
                  // And there is a note detected, switch to "has note" state
                  currentState = LightState.HAS_NOTE;
                } else {
                  // Otherwise, go back to the "no note" state.
                  currentState = LightState.NO_NOTE;
                }
              }
            }
            case HAS_NOTE -> {
              if (isAiming.getAsBoolean()) {
                // If we're aiming, switch to the "aiming" state.
                currentState = LightState.AIMING;
              }
            }
            case AIMING -> {
              if (isAimOK.getAsBoolean()) {
                // If the aiming is within tolerance, switch to the "aim ok" state
                currentState = LightState.AIM_OK;
              } else if (!isAiming.getAsBoolean()) {
                // If the aiming gets canceled, go back to the "has note" state
                currentState = LightState.HAS_NOTE;
              }
            }
            case AIM_OK -> {
              if (!isAiming.getAsBoolean()) {
                // If the aiming gets canceled, go back to the "has note" state
                currentState = LightState.HAS_NOTE;
              } else if (!isAimOK.getAsBoolean()) {
                // If the aiming falls out of tolerance, switch back to the "aiming" state
                currentState = LightState.AIMING;
              } else if (isShooting.getAsBoolean()) {
                // If we're shooting, go to the "shooting" state.
                currentState = LightState.SHOOTING;
              }
            }
            case SHOOTING -> {
              if (!isAimOK.getAsBoolean()) {
                // If the aiming falls out of tolerance, switch back to the "aiming" state
                currentState = LightState.AIMING;
              } else if (!isShooting.getAsBoolean()) {
                // If we stop shooting
                if (!hasNote.getAsBoolean()) {
                  // And the note successfully leaves the shooter, switch to the "no note" state.
                  currentState = LightState.NO_NOTE;
                } else {
                  // Otherwise, go back to the "aim ok" state.
                  currentState = LightState.AIM_OK;
                }
              }
            }
          }
        });
  }
}
