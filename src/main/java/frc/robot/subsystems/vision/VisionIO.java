package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public class VisionIOInputs {
        public double x = 0;
        public double y = 0;
        public double rot = 0;
        public double timestamp = 0;
        public int trackedTags = 0;
        public boolean isNewData = false;
    }

    public default void updateInputs(VisionIOInputs inputs) {
    };

    public default String getName() {
        return "";
    }
}
