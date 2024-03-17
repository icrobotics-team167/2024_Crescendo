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

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.Mode;
import frc.robot.subsystems.vision.interfaceLayers.*;
import frc.robot.util.MathUtils;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  private Consumer<VisionPoseEstimate> estimationConsumer;
  private VisionIO[] cameras;
  private VisionIOInputsAutoLogged[] cameraData;

  public VisionSubsystem(Consumer<VisionPoseEstimate> estimationConsumer) {
    this.estimationConsumer = estimationConsumer;

    if (Robot.currentMode != Mode.REAL) {
      cameras = new VisionIO[] {new VisionIO() {}};
    } else {
      cameras =
          new VisionIO[] {
            new VisionIOPhoton(
                "Camera_Module_v1",
                new Transform3d(
                    Meters.convertFrom(11.75, Inches),
                    Meters.convertFrom(-22.75, Centimeters),
                    Meters.convertFrom(28.5, Centimeters),
                    new Rotation3d(0, Radians.convertFrom(-45, Degrees), 0)))
          };
    }

    cameraData = new VisionIOInputsAutoLogged[cameras.length];

    for (int i = 0; i < cameraData.length; i++) {
      cameraData[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < cameraData.length; i++) {
      cameras[i].updateInputs(cameraData[i]);
      Logger.processInputs("VisionPoseEstimator/" + cameras[i].getName(), cameraData[i]);
    }
  }

  public void updateEstimation() {
    for (int i = 0; i < cameraData.length; i++) {
      if (cameraData[i].isNewData) {
        double trustworthiness =
            calculateTrustworthiness(cameraData[i].trackedTags, cameraData[i].poseEstimate);
        Logger.recordOutput(
            "VisionPoseEstimator/" + cameras[i].getName() + "/trustworthiness", trustworthiness);
        VisionPoseEstimate estimate =
            new VisionPoseEstimate(
                cameraData[i].poseEstimate,
                trustworthiness,
                trustworthiness,
                cameraData[i].timestamp);
        estimationConsumer.accept(estimate);
      }
    }
  }

  private double calculateTrustworthiness(Pose3d[] tagPoses, Pose2d botPose) {
    if (tagPoses.length == 1) {
      return .25
          + .1
              * tagPoses[0]
                  .getTranslation()
                  .getDistance(new Translation3d(botPose.getX(), botPose.getY(), 0));
    }
    double[] tagDistances = new double[tagPoses.length];
    for (int i = 0; i < tagDistances.length; i++) {
      tagDistances[i] =
          tagPoses[i]
              .getTranslation()
              .getDistance(new Translation3d(botPose.getX(), botPose.getY(), 0));
    }
    return .1 / tagPoses.length + .01 * MathUtils.average(tagDistances);
  }

  public class VisionPoseEstimate {
    public VisionPoseEstimate(
        Pose2d poseEstimate,
        double translationalTrustworthinessMeters,
        double rotationalTrustworthinessRadians,
        double timestamp) {
      this.poseEstimate = poseEstimate;
      this.trustworthiness =
          VecBuilder.fill(
              translationalTrustworthinessMeters,
              translationalTrustworthinessMeters,
              rotationalTrustworthinessRadians);
      this.timestamp = timestamp;
    }

    public Pose2d poseEstimate;
    public Matrix<N3, N1> trustworthiness;
    public double timestamp;
  }
}
