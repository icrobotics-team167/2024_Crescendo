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

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import org.ejml.simple.UnsupportedOperation;

public class SwerveUtils {
  private SwerveUtils() {
    throw new UnsupportedOperation("This is a utility class!");
  }

  public static double[] queueToDoubleArray(Queue<Double> queue) {
    double[] array = new double[queue.size()];
    for (int i = 0; i < array.length; i++) {
      array[i] = queue.poll();
    }
    return array;
  }

  public static Rotation2d[] queueToRotation2dArray(Queue<Double> queue) {
    Rotation2d[] array = new Rotation2d[queue.size()];
    for (int i = 0; i < array.length; i++) {
      array[i] = Rotation2d.fromRotations(queue.poll());
    }
    return array;
  }
}
