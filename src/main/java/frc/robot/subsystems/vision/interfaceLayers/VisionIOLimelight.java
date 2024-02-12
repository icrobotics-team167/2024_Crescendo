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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Field;
import frc.robot.subsystems.vision.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
  private String name = "";

  public VisionIOLimelight(String name) {
    this.name = name;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isNewData = false;
    // If the Limelight doesn't see any tags to track, stop.
    if (!LimelightHelpers.getTV(name)) {
      return;
    }
    Pose2d poseEstimate = LimelightHelpers.getBotPose2d_wpiBlue(name);
    // If the Limelight returns a blank pose, stop.
    // TODO: This might be redundant?
    if (poseEstimate.equals(new Pose2d())) {
      return;
    }
    // If the pose is outside the field, it's obviously a bad pose so stop.
    if (poseEstimate.getX() < 0
        || poseEstimate.getY() < 0
        || poseEstimate.getX() > Field.FIELD_LENGTH.in(Meters)
        || poseEstimate.getY() > Field.FIELD_WIDTH.in(Meters)) {
      return;
    }

    // If all checks succeed, then write data.
    inputs.poseEstimate = poseEstimate;
    inputs.isNewData = true;
    inputs.timestamp =
        Timer.getFPGATimestamp()
            - ((LimelightHelpers.getLatency_Capture(name)
                    + LimelightHelpers.getLatency_Pipeline(name))
                / 1000);
  }
}
