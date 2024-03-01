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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.interfaceLayers.FeederIO;
import frc.robot.subsystems.shooter.interfaceLayers.FlywheelIO;
import frc.robot.subsystems.shooter.interfaceLayers.IntakeIO;
import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIO;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** A class containing all the logic and commands to make the shooter mechanism work. */
public class Shooter {
  private final FlywheelSubsystem flywheel;
  private final PivotSubsystem pivot;
  private final NoteDetectorSubsystem noteDetector;
  private final IntakeSubsystem intake;
  private final FeederSubsystem feeder;

  private PIDController autoAimController;

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

    autoAimController = new PIDController(1.0 / 50, 0, 1.0 / 200);
    autoAimController.enableContinuousInput(-180, 180);
  }

  public Command intake() {
    return intake.getIntakeCommand();
  }

  public Command intakeOut() {
    return intake.getIntakeOutCommand().alongWith(feeder.getUnfeedCommand());
  }

  public Command autoIntake() {
    if (!noteDetector.hasNote()) {
      return parallel(intake.getIntakeCommand(), feeder.getFeedCommand())
          .until(noteDetector::hasNote)
          .finallyDo(() -> feeder.getUnfeedCommand());
    } else {
      return null;
    }
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
        // pivot.getPivotCommand(
        //     () -> {
        //       // return SpencerAim(drivebase);
        //       return TadaAim(drivebase);
        //       // return null;
        //     }),
        drivebase.getDriveCommand(
            xVel,
            yVel,
            () -> {
              // return SpencerYaw(drivebase);
              return TadaYaw(drivebase);
              // return 0;
              // Michael was here
            }));
  }

  private Rotation2d SpencerAim(SwerveSubsystem drivebase) {
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
  }

  private double speakerY = 5.5;
  private double speakerZ = 2;

  // ARM ANGLE MATH
  private Rotation2d TadaAim(SwerveSubsystem drivebase) {
    double speakerX = Robot.isOnRed() ? Field.FIELD_LENGTH.in(Meters) : 0;
    Translation2d currentBotPosition = drivebase.getPose().getTranslation();
    double targetDistance = currentBotPosition.getDistance(new Translation2d(speakerX, speakerY));
    return new Rotation2d(Math.atan(speakerZ / targetDistance));
  }

  private double SpencerYaw(SwerveSubsystem drivebase) {
    double tx = drivebase.visionPoseEstimator.getTX();
    return tx / -75.0;
  }

  // ROBOT ROTATE MATH
  private double TadaYaw(SwerveSubsystem drivebase) {
    double speakerX = Robot.isOnRed() ? Field.FIELD_LENGTH.in(Meters) : 0;
    Translation2d currentBotPosition = drivebase.getPose().getTranslation();
    Rotation2d currentBotYaw = drivebase.getPose().getRotation();
    Rotation2d targetBotYaw =
        new Translation2d(speakerX, speakerY).minus(currentBotPosition).getAngle();
    return autoAimController.calculate(currentBotYaw.getDegrees(), targetBotYaw.getDegrees());
  }
}
