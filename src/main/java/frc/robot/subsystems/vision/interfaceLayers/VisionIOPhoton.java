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

package frc.robot.subsystems.vision.interfaceLayers;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Field;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionIOPhoton implements VisionIO {
  private String name = "";
  private PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;
  private Transform3d robotToCameraTransform;

  public VisionIOPhoton(String name, Transform3d robotToCameraTransform) {
    this.name = name;
    this.robotToCameraTransform = robotToCameraTransform;
    camera = new PhotonCamera(name);
    try {
      poseEstimator =
          new PhotonPoseEstimator(
              AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              camera,
              robotToCameraTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (Exception e) {
      DriverStation.reportError(
          "PhotonVision failed to load camera " + name + "! Failed with error " + e.getMessage(),
          false);
      poseEstimator = null;
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // inputs = new VisionIOInputs();
    // If the camera isn't connected, stop.
    if (camera.isConnected() == false) {
      // System.out.println("Camera connected check failed");
      inputs.statusCode = VisionStatusCode.CAMERA_FAIL;
      return;
    }

    // System.out.println("Camera connected check succeeded");

    // If the pose estimator failed to load, stop.
    if (poseEstimator == null) {
      inputs.statusCode = VisionStatusCode.ESTIMATOR_FAIL;
      // System.out.println("Estimator wasn't initialized");
      return;
    }

    // System.out.println("Estimator was initialized");

    Optional<EstimatedRobotPose> data = poseEstimator.update();
    // If the pose estimator doesn't have any data, stop.
    if (data.isEmpty()) {
      inputs.statusCode = VisionStatusCode.NO_DATA;
      return;
    }

    EstimatedRobotPose botPoseEstimate = data.get();
    if (botPoseEstimate.strategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
        && botPoseEstimate.targetsUsed.get(0).getPoseAmbiguity() > .2) {
      inputs.statusCode = VisionStatusCode.BAD_TAG;
      return;
    }
    // If the pose is outside the field, it's obviously a bad pose so stop.
    if (botPoseEstimate.estimatedPose.getX() < 0
        || botPoseEstimate.estimatedPose.getY() < 0
        || botPoseEstimate.estimatedPose.getX() > Field.FIELD_LENGTH.in(Meters)
        || botPoseEstimate.estimatedPose.getY() > Field.FIELD_WIDTH.in(Meters)) {
      inputs.statusCode = VisionStatusCode.BAD_POSE;
      return;
    }

    // If all checks succeed, then write data.
    inputs.statusCode = VisionStatusCode.OK;
    inputs.poseEstimate = botPoseEstimate.estimatedPose.toPose2d();
    inputs.timestamp = botPoseEstimate.timestampSeconds;
    inputs.trackedTags = new Pose3d[botPoseEstimate.targetsUsed.size()];
    for (int i = 0; i < inputs.trackedTags.length; i++) {
      inputs.trackedTags[i] =
          new Pose3d(inputs.poseEstimate)
              .plus(robotToCameraTransform)
              .plus(botPoseEstimate.targetsUsed.get(i).getBestCameraToTarget());
    }
  }

  @Override
  public String getName() {
    return name;
  }
}
