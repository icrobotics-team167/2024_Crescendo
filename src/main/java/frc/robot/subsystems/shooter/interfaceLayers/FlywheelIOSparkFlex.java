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

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.util.SparkUtils;

public class FlywheelIOSparkFlex implements FlywheelIO {
  private final CANSparkFlex topFlywheel;
  private final CANSparkFlex bottomFlywheel;

  public FlywheelIOSparkFlex() {
    topFlywheel = new CANSparkFlex(0, MotorType.kBrushless); // TODO: Configure
    bottomFlywheel = new CANSparkFlex(0, MotorType.kBrushless);
    SparkUtils.configureSettings(false, IdleMode.kCoast, Amps.of(60), bottomFlywheel);
    SparkUtils.configureSettings(true, IdleMode.kCoast, Amps.of(60), bottomFlywheel);
  }

  @Override
  public void runSpeaker() {
    bottomFlywheel.setVoltage(12);
    topFlywheel.setVoltage(12);
  }

  @Override
  public void runAmp() {
    bottomFlywheel.setVoltage(6);
    topFlywheel.setVoltage(-6);
  }

  @Override
  public void stop() {
    topFlywheel.setVoltage(0);
    bottomFlywheel.setVoltage(0);
  }
}
