package frc.robot.subsystems.vision;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionPoseEstimator extends SubsystemBase {
    private BiConsumer<Pose2d, Double> estimationConsumer;
    private Supplier<Pose2d> currentEstimateSupplier;
    private VisionIO[] cameras;
    private VisionIOInputsAutoLogged[] cameraData;

    public VisionPoseEstimator(BiConsumer<Pose2d, Double> estimationConsumer, Supplier<Pose2d> currentEstimateSupplier) {
        this.estimationConsumer = estimationConsumer;
        this.currentEstimateSupplier = currentEstimateSupplier;

        cameras = new VisionIO[] {
                new PhotonVisionIO("AprilTagLL", new Transform3d())
        };

        cameraData = new VisionIOInputsAutoLogged[cameras.length];
        for (int i = 0; i < cameraData.length; i++) {
            cameraData[i] = new VisionIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < cameraData.length; i++) {
            cameras[i].updateInputs(cameraData[i], currentEstimateSupplier.get());
            Logger.processInputs("VisionPoseEstimator/" + cameras[i].getName(), cameraData[i]);
        }
    }

    public void updateEstimation() {
        for (int i = 0; i < cameraData.length; i++) {
            estimationConsumer.accept(
                    new Pose2d(cameraData[i].x, cameraData[i].y, new Rotation2d(cameraData[i].rot)),
                    cameraData[i].timestamp);
        }
    }
}
