package frc.robot.subsystems.misc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.misc.interfaceLayers.LightsIO;
import frc.robot.subsystems.misc.interfaceLayers.LightsIOBlinkin;
import frc.robot.subsystems.misc.interfaceLayers.LightsIOBlinkin.Colours;
import frc.robot.util.CANConstants;

public class LightSubsystem extends SubsystemBase {

  LightsIO light;
  public LightSubsystem(LightsIO io) {
    light = io;

  }

  public void setColor(Colours color) {
    light.setColor(color);
  }

  public void setColorValue(int num) {
    light.setColorValue(num);
  }

  public void setColorNull() {
    light.setColorValue(0);
  }
}
