package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Lights;
import frc.robot.misc.Lights.Colours;

public class LightSubsystem extends SubsystemBase {
    Lights lights;
    public LightSubsystem() {
        lights = new Lights(8); // TODO: Configure ID
    }

    public void setColour(Colours colour) {
        lights.setColour(colour);
    }

    public void setColorValue(int num) {
        lights.setColorValue(num);
    }
}
