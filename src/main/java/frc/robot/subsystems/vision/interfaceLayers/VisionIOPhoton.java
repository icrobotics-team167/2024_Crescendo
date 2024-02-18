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

  public VisionIOPhoton(String name, Transform3d robotToCameraTransform) {
    this.name = name;
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
      DriverStation.reportError("PhotonVision failed to load camera " + name + "!", false);
      poseEstimator = null;
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isNewData = false;
    // If the camera didn't load properly, stop.
    if (poseEstimator == null) {
      return;
    }

    Optional<EstimatedRobotPose> data = poseEstimator.update();
    // If the pose estimator doesn't have any data, stop.
    if (data.isEmpty()) {
      return;
    }

    EstimatedRobotPose botPoseEstimate = data.get();
    // If multi-tag tracking fails and the pose ambiguity score of the single tag
    // tracking is too large, stop.
    if (botPoseEstimate.strategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
        && botPoseEstimate.targetsUsed.get(0).getPoseAmbiguity() > 0.2) {
      return;
    }
    // If the pose is outside the field, it's obviously a bad pose so stop.
    if (botPoseEstimate.estimatedPose.getX() < 0
        || botPoseEstimate.estimatedPose.getY() < 0
        || botPoseEstimate.estimatedPose.getX() > Field.FIELD_LENGTH.in(Meters)
        || botPoseEstimate.estimatedPose.getY() > Field.FIELD_WIDTH.in(Meters)) {
      return;
    }

    // If all checks succeed, then write data.
    inputs.isNewData = true;
    inputs.poseEstimate = botPoseEstimate.estimatedPose.toPose2d();
    inputs.timestamp = botPoseEstimate.timestampSeconds;
    inputs.trackedTags = new Transform3d[botPoseEstimate.targetsUsed.size()];
    for (int i = 0; i < inputs.trackedTags.length; i++) {
      inputs.trackedTags[i] = botPoseEstimate.targetsUsed.get(i).getBestCameraToTarget();
    }
    inputs.tX = botPoseEstimate.targetsUsed.get(0).getYaw();
    inputs.tY = botPoseEstimate.targetsUsed.get(0).getPitch();
  }

  @Override
  public String getName() {
    return name;
  }
}
