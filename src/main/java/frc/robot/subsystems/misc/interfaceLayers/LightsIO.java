package frc.robot.subsystems.misc.interfaceLayers;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.misc.interfaceLayers.*;
import frc.robot.subsystems.misc.interfaceLayers.LightsIOBlinkin.Colours;

public interface LightsIO {

  public class LightsIOInputs {
    //TODO: Implement
  }

  public default void updateInputs(LightsIOInputs inputs) {}

  public default void setColor(Colours color) {}

  public default void setColorValue(int num) {}

  public default void setColorNull() {}

  
}
