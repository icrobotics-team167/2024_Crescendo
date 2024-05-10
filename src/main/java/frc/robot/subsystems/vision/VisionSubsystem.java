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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.Mode;
import frc.robot.subsystems.vision.interfaceLayers.*;
import frc.robot.subsystems.vision.interfaceLayers.VisionIO.VisionPoseEstimate;
import frc.robot.subsystems.vision.interfaceLayers.VisionIO.VisionStatusCode;
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
                    Meters.convertFrom(12.75, Inches),
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
      if (cameraData[i].statusCode == VisionStatusCode.OK) {
        double trustWorthiness =
            calculateStDevs(cameraData[i].trackedTags, cameraData[i].poseEstimate);
        estimationConsumer.accept(
            new VisionPoseEstimate(
                cameraData[i].poseEstimate,
                trustWorthiness,
                trustWorthiness,
                cameraData[i].timestamp));
      }
    }
  }

  public Rotation2d getTX(Pose2d botPose) {
    for (VisionIOInputsAutoLogged data : cameraData) {
      if (data.isNewData) {
        return data.trackedTags[0]
            .toPose2d()
            .getTranslation()
            .minus(botPose.getTranslation())
            .getAngle()
            .minus(botPose.getRotation());
      }
    }
    return Rotation2d.fromDegrees(0);
  }

  private double calculateStDevs(Pose3d[] tagPoses, Pose2d botPose) {
    switch (tagPoses.length) {
      case 0 -> {
        return .9;
      }
      case 1 -> {
        return Meters.convertFrom(
            20 // Base standard deviation
                + 1 * calculateBotToTagDist(tagPoses[0], botPose), // Scaled with distance
            Millimeters);
      }
      default -> {
        return Meters.convertFrom(
            10 // Base standard deviation
                + 10 / tagPoses.length // Inversely scaled with number of tags
                + .5 * averageBotToTagDist(tagPoses, botPose), // Scaled with avg distance
            Millimeters);
      }
    }
  }

  private double calculateBotToTagDist(Pose3d tagPose, Pose2d botPose) {
    return tagPose
        .getTranslation()
        .getDistance(new Translation3d(botPose.getX(), botPose.getY(), 0));
  }

  private double averageBotToTagDist(Pose3d[] tagPoses, Pose2d botPose) {
    double sum = 0;
    for (Pose3d pose : tagPoses) {
      sum += calculateBotToTagDist(pose, botPose);
    }
    return sum / tagPoses.length;
  }
}
