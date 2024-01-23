package frc.robot.subsystems.vision;

import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionPoseEstimator {
    private BiConsumer<Pose2d, Double> estimationConsumer;
    private VisionIO[] cameras;
    private VisionIOInputs[] cameraData;

    public VisionPoseEstimator(BiConsumer<Pose2d, Double> estimationConsumer) {
        this.estimationConsumer = estimationConsumer;

        cameras = new VisionIO[] {
                new PhotonVisionIO("AprilTagLL", new Transform3d())
        };

        cameraData = new VisionIOInputs[cameras.length];
        for (int i = 0; i < cameraData.length; i++) {
            cameraData[i] = new VisionIOInputsAutoLogged();
        }
    }

    public void updateEstimation() {
        for (int i = 0; i < cameraData.length; i++) {
            cameras[i].updateInputs(cameraData[i]);
            estimationConsumer.accept(
                    new Pose2d(cameraData[i].x, cameraData[i].y, new Rotation2d(cameraData[i].rot)),
                    cameraData[i].timestamp);
        }
    }
}
