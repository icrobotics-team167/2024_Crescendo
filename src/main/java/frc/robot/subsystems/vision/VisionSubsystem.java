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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.Mode;
import frc.robot.subsystems.vision.interfaceLayers.*;
import frc.robot.subsystems.vision.interfaceLayers.VisionIO.VisionPoseEstimate;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  private Consumer<VisionPoseEstimate> estimationConsumer;
  private VisionIO[] cameras;
  private VisionIOInputsAutoLogged[] cameraData;

  public VisionSubsystem(Consumer<VisionPoseEstimate> estimationConsumer) {
    this.estimationConsumer = estimationConsumer;

    if (Robot.currentMode == Mode.SIM) {
      cameras = new VisionIO[] {new VisionIO() {}};
    } else {
      cameras =
          new VisionIO[] {
            new VisionIOPhoton(
                "Camera_Module_v1",
                new Transform3d(
                    Meters.convertFrom(10, Inches),
                    Meters.convertFrom(-8.5, Inches),
                    Meters.convertFrom(10, Inches),
                    new Rotation3d(0, Radians.convertFrom(-30, Degrees), 0)))
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
        VisionPoseEstimate estimate = new VisionPoseEstimate();
        estimate.poseEstimate = cameraData[i].poseEstimate;
        estimate.trustworthiness =
            VecBuilder.fill(
                cameraData[i].translationalTrustworthinessMeters,
                cameraData[i].translationalTrustworthinessMeters,
                cameraData[i].rotationalTrustworthinessRadians);
        estimate.timestamp = cameraData[i].timestamp;
        estimationConsumer.accept(estimate);
      }
    }
  }
}
