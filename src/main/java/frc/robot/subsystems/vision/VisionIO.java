package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

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

    public default void updateInputs(VisionIOInputs inputs, Pose2d currentPose) {
    };

    public default String getName() {
        return "";
    }
}
