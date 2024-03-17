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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public class VisionIOInputs {
    public Pose2d poseEstimate = new Pose2d();
    public Pose3d[] trackedTags = new Pose3d[0];
    public double timestamp = 0;
    public boolean isNewData = false;
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

  public default void updateInputs(VisionIOInputs inputs) {}

  public default String getName() {
    return "";
  }
}
