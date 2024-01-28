package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public class VisionIOInputs {
        public Pose2d poseEstimate = new Pose2d();
        public double timestamp = 0;
        public boolean isNewData = false;
    }

    public default void updateInputs(VisionIOInputs inputs, Pose2d currentPose) {
    };

    public default String getName() {
        return "";
    }
}
