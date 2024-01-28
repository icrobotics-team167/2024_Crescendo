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

import edu.wpi.first.wpilibj.Timer;

public class VisionIOLimelight implements VisionIO {
    private String name = "";

    public VisionIOLimelight(String name) {
        this.name = name;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        if (!LimelightHelpers.getTV(name)) {
            inputs.isNewData = false;
            return;
        }
        inputs.isNewData = true;
        inputs.poseEstimate = LimelightHelpers.getBotPose2d_wpiBlue(name);
        inputs.timestamp = Timer.getFPGATimestamp()
                - ((LimelightHelpers.getLatency_Capture(name) + LimelightHelpers.getLatency_Pipeline(name))
                        / 1000);
    }
}
