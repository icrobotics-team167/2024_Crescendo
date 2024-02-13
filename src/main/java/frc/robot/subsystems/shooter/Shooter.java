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

package frc.robot.subsystems.shooter;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.interfaceLayers.FeederIO;
import frc.robot.subsystems.shooter.interfaceLayers.FlywheelIO;
import frc.robot.subsystems.shooter.interfaceLayers.IntakeIO;
import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIO;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIO;
import java.util.function.DoubleSupplier;

/** A class containing all the logic and commands to make the shooter mechanism work. */
public class Shooter {
  private final FlywheelSubsystem flywheel;
  private final PivotSubsystem pivot;
  private final NoteDetectorSubsystem noteDetector;
  private final IntakeSubsystem intake;
  private final FeederSubsystem feeder;

  public Shooter(
      FeederIO feederIO,
      FlywheelIO flywheelIO,
      PivotIO pivotIO,
      NoteDetectorIO noteDetectorIO,
      IntakeIO intakeIO) {
    // TODO: Implement flywheel and pivot interfaces
    flywheel = new FlywheelSubsystem(flywheelIO);
    pivot = new PivotSubsystem(pivotIO);
    noteDetector = new NoteDetectorSubsystem(noteDetectorIO);
    intake = new IntakeSubsystem(intakeIO);
    feeder = new FeederSubsystem(feederIO);
  }

  public Command intake() {
    System.out.println("running no sight");
    return intake.getIntakeCommand();
  }

  public Command autoIntake() {
    System.out.println("runnong sight");
    return parallel(intake.getIntakeCommand(), feeder.getFeedCommand())
        .until(noteDetector::hasNote);
  }

  public Command getManualControlCommand(DoubleSupplier pivotSupplier) {
    return pivot.getManualOverrideCommand(pivotSupplier);
  }

  public Command getPivotRestingPositionCommand() {
    return pivot.getRestingPositionCommand();
  }

  public Command getAmpShotCommand() {
    return parallel(
            parallel(
                // Get up to 90 degrees pivot
                pivot.getPivotCommand(() -> Rotation2d.fromDegrees(90)),
                // Spin up flywheels
                flywheel.getAmpShotCommand()),
            // Once the flywheels are up to speed and the pivot is at the setpoint, feed the note
            waitUntil(
                    () ->
                        flywheel.isUpToSpeed() && Math.abs(pivot.getAngle().getDegrees() - 90) < 2)
                .andThen(feeder.getFeedCommand()))
        .until(() -> !noteDetector.hasNote()); // Stop when note is launched
  }

  public Command getPivotVelSysIdCommand() {
    return pivot.getPivotVelSysID();
  }

  public void setPivotDefaultCommand(Command command) {
    pivot.setDefaultCommand(command);
  }

  public Command feed() {
    return feeder.getFeedCommand();
  }
}
