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
import frc.robot.subsystems.swerve.SwerveSubsystem;
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
    return intake.getIntakeCommand();
  }

  public Command intakeOut() {
    return intake.getIntakeOutCommand();
  }

  public Command autoIntake() {
    return parallel(intake.getIntakeCommand(), feeder.getFeedCommand())
        .until(noteDetector::hasNote)
        .andThen(feeder.getUnfeedCommand().withTimeout(0.2));
  }

  public Command getManualControlCommand(DoubleSupplier pivotSupplier) {
    return pivot.getManualOverrideCommand(pivotSupplier);
  }

  public Command getPivotRestingPositionCommand() {
    return pivot.getRestingPositionCommand();
  }

  public Command getAmpShotCommand() {
    return flywheel.getAmpShotCommand();
    // return parallel(
    //         parallel(
    //             // Get up to 90 degrees pivot
    //             pivot.getPivotCommand(() -> Rotation2d.fromDegrees(90)),
    //             // Spin up flywheels
    //             flywheel.getAmpShotCommand()),
    //         // Once the flywheels are up to speed and the pivot is at the setpoint, feed the note
    //         waitUntil(
    //                 () ->
    //                     flywheel.isUpToSpeed() && Math.abs(pivot.getAngle().getDegrees() - 90) <
    // 2)
    //             .andThen(feeder.getFeedCommand()))
    //     .until(() -> !noteDetector.hasNote()); // Stop when note is launched
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
  // This is a temp thing for testing stuff
  public Command shoot() {
    return flywheel.getSpeakerShotCommand();
  }

  public Command getSpeakerShotCommand() {
    return parallel(
            parallel(
                pivot.getPivotCommand(() -> Rotation2d.fromDegrees(30)),
                flywheel.getSpeakerShotCommand()),
            waitUntil(() -> flywheel.isUpToSpeed()).andThen(feeder.getFeedCommand()))
        .until(() -> !noteDetector.hasNote());
  }

  public Command getTeleopAutoAimCommand(
      SwerveSubsystem drivebase, DoubleSupplier xVel, DoubleSupplier yVel) {
    return parallel(
            pivot.getPivotCommand(
                () -> {
                  // Magic code Spencer and Chris wrote
                  // TODO: Fix scalar
                  double ty = drivebase.visionPoseEstimator.getTY();
                  double MIN_ANGLE = 0;
                  double MAX_ANGLE = 60;
                  double MAX_TY = 32.1;
                  double MIN_TY = 0;
                  double SCALAR = 1;

                  double angle = SCALAR * (((ty - MIN_TY) * ((MAX_ANGLE - MIN_ANGLE) / MAX_TY)) - MIN_ANGLE);
                  return Rotation2d.fromDegrees(angle);
                }),
            drivebase.getDriveCommand(
                xVel,
                yVel,
                () -> {
                  double tx = drivebase.visionPoseEstimator.getTX();
                  return tx / -75.0;
                }),
            flywheel.getSpeakerShotCommand(),
            waitUntil(flywheel::isUpToSpeed).andThen(feeder.getFeedCommand()))
        .until(() -> !noteDetector.hasNote());
  }
}
