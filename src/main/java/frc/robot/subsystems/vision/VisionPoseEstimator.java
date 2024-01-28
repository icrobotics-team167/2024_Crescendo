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

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.Mode;

public class VisionPoseEstimator extends SubsystemBase {
    private BiConsumer<Pose2d, Double> estimationConsumer;
    private Supplier<Pose2d> currentEstimateSupplier;
    private VisionIO[] cameras;
    private VisionIOInputsAutoLogged[] cameraData;

    public VisionPoseEstimator(BiConsumer<Pose2d, Double> estimationConsumer,
            Supplier<Pose2d> currentEstimateSupplier) {
        this.estimationConsumer = estimationConsumer;
        this.currentEstimateSupplier = currentEstimateSupplier;

        if (Robot.currentMode == Mode.SIM) {
            cameras = new VisionIO[] {
                    new VisionIO() {
                    }
            };
        } else {
            cameras = new VisionIO[] {
                    new PhotonVisionIO("AprilTagLL", new Transform3d())
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
            cameras[i].updateInputs(cameraData[i], currentEstimateSupplier.get());
            Logger.processInputs("VisionPoseEstimator/" + cameras[i].getName(), cameraData[i]);
        }
    }

    public void updateEstimation() {
        for (int i = 0; i < cameraData.length; i++) {
            if (cameraData[i].isNewData) {
                estimationConsumer.accept(
                        cameraData[i].poseEstimate,
                        cameraData[i].timestamp);
            }
        }
    }
}
