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

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

public class PhotonVisionIO implements VisionIO {
    private String name = "";
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    public PhotonVisionIO(String name, Transform3d robotToCameraTransform) {
        this.name = name;
        camera = new PhotonCamera(name);
        try {
            poseEstimator = new PhotonPoseEstimator(
                    AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCameraTransform);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (Exception e) {
            DriverStation.reportError("PhotonVision failed to load the AprilTag map!", false);
            poseEstimator = null;
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d currentPose) {
        inputs.isNewData = false;
        if (poseEstimator == null) {
            return;
        }

        Optional<EstimatedRobotPose> data = poseEstimator.update();
        if (data.isEmpty()) {
            return;
        }

        EstimatedRobotPose botPoseEstimate = data.get();
        if (distanceFromCurrentPose(currentPose, botPoseEstimate.estimatedPose) > 1) {
            return;
        }
        inputs.isNewData = true;
        inputs.poseEstimate = botPoseEstimate.estimatedPose.toPose2d();
        inputs.timestamp = botPoseEstimate.timestampSeconds;
    }

    @Override
    public String getName() {
        return name;
    }

    private double distanceFromCurrentPose(Pose2d currentPose, Pose3d newPose) {
        Pose2d newPose2d = newPose.toPose2d();
        return currentPose.minus(newPose2d).getTranslation().getNorm();
    }
}
