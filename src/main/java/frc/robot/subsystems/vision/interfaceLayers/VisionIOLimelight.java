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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.Field;
import org.littletonrobotics.junction.Logger;

public class VisionIOLimelight implements VisionIO {
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // inputs = new VisionIOInputs();
    inputs.trackedTags = new Pose3d[0];
    // If the Limelight doesn't see any tags to track, stop.
    if (!hasTracking()) {
      return;
    }

    double[] poseArray =
        NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("botpose_wpiblue")
            .getDoubleArray(new double[6]);

    // If the pose is outside the field, it's obviously a bad pose so stop.
    if (poseArray[0] < 0
        || poseArray[1] < 0
        || poseArray[0] > Field.FIELD_LENGTH.in(Meters)
        || poseArray[1] > Field.FIELD_WIDTH.in(Meters)) {
      return;
    }

    // If all checks succeed, then write data.
    inputs.poseEstimate =
        new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
    inputs.statusCode = VisionStatusCode.OK;
    inputs.timestamp = getTimeStamp();
  }

  private double getTimeStamp() {
    return Logger.getRealTimestamp()
        + ((NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0)
                + NetworkTableInstance.getDefault()
                    .getTable("limelight")
                    .getEntry("cl")
                    .getDouble(0))
            / 1000.0);
  }

  private boolean hasTracking() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0)
        == 1.0;
  }
}
