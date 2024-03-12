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
import frc.robot.subsystems.misc.interfaceLayers.LightsIOBlinkin.Colors;
import frc.robot.subsystems.misc.interfaceLayers.LightsIOInputsAutoLogged;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class LightSubsystem extends SubsystemBase {
  LightsIO io;
  LightsIOInputsAutoLogged inputs = new LightsIOInputsAutoLogged();

  public LightSubsystem(LightsIO io) {
    this.io = io;
    io.setColor(Colors.GREEN);
  }

  @Override
  public void periodic() {
    io.setColorValue(currentState.colorValue);
    io.updateInputs(inputs);
    Logger.processInputs("Misc/lights", inputs);
  }

  private enum LightState {
    NO_NOTE(0),
    INTAKING(0),
    HAS_NOTE(0),
    AIMING(0),
    AIM_OK(0),
    SHOOTING(0);

    public int colorValue;

    private LightState(int colorValue) {
      this.colorValue = colorValue;
    }
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
          if (isIntaking.getAsBoolean()) {
            currentState = LightState.INTAKING;
          } else {
            if (hasNote.getAsBoolean()) {
              if (isAiming.getAsBoolean()) {
                if (isAimOK.getAsBoolean()) {
                  if (isShooting.getAsBoolean()) {
                    currentState = LightState.SHOOTING;
                  } else {
                    currentState = LightState.AIM_OK;
                  }
                } else {
                  currentState = LightState.AIMING;
                }
              } else {
                currentState = LightState.HAS_NOTE;
              }
            } else {
              currentState = LightState.NO_NOTE;
            }
          }
        });
  }
}
