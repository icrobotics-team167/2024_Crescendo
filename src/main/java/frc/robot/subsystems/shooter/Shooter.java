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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.Robot;
import frc.robot.subsystems.misc.LightSubsystem;
import frc.robot.subsystems.misc.interfaceLayers.*;
import frc.robot.subsystems.shooter.interfaceLayers.ClimberIO;
import frc.robot.subsystems.shooter.interfaceLayers.FeederIO;
import frc.robot.subsystems.shooter.interfaceLayers.FlywheelIO;
import frc.robot.subsystems.shooter.interfaceLayers.IntakeIO;
import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIO;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** A class containing all the logic and commands to make the shooter mechanism work. */
public class Shooter {
  private final FlywheelSubsystem flywheel;
  private final PivotSubsystem pivot;
  private final NoteDetectorSubsystem noteDetector;
  private final IntakeSubsystem intake;
  private final FeederSubsystem feeder;
  private final LightSubsystem light;
  private final ClimberSubsystem climb;

  private InterpolatingDoubleTreeMap fudgeFactorLerpTable;

  private PIDController speakerYawPidController;

  public Shooter(
      FeederIO feederIO,
      FlywheelIO flywheelIO,
      PivotIO pivotIO,
      NoteDetectorIO noteDetectorIO,
      IntakeIO intakeIO,
      LightsIO lightIO,
      ClimberIO climberIO) {
    flywheel = new FlywheelSubsystem(flywheelIO);
    pivot = new PivotSubsystem(pivotIO);
    noteDetector = new NoteDetectorSubsystem(noteDetectorIO);
    intake = new IntakeSubsystem(intakeIO);
    feeder = new FeederSubsystem(feederIO);
    light = new LightSubsystem(lightIO);
    climb = new ClimberSubsystem(climberIO);

    light.setDefaultCommand(
        light.setState(
            noteDetector::hasNoteInIntake, // Is the bot currently intaking a note?
            noteDetector::hasNoteInShooter, // Does the bot have a note in the shooter?
            intake::isRunning, // Is the bot intaking?
            () -> // Is the pivot aiming?
            pivot.getCurrentCommand() != null
                    && pivot.getCurrentCommand().getName().equals("Pivot to angle"),
            pivot::isAtSetpoint, // Is the pivot at its setpoint?
            flywheel::isUpToSpeed)); // Is it shooting?

    // Lerped fudge factor for pivot aiming to account for gravity
    // Is added to a tan^-1
    // Known good angles:
    // 48.4 degrees for at subwoofer measured 47.3
    // 32 degrees for at podium
    // 38.5 degrees at x;2.15, y:5.54
    fudgeFactorLerpTable = new InterpolatingDoubleTreeMap();
    fudgeFactorLerpTable.put(0.0, 0.0);
    fudgeFactorLerpTable.put(1.0, 2.5);
    fudgeFactorLerpTable.put(3.0, 6.65);

    speakerYawPidController = new PIDController(0.5, 0, 0);
    speakerYawPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public Command intakeOut() {
    return parallel(
        intake.getIntakeOutCommand(), feeder.getUnfeedCommand(), flywheel.getSourceIntakeCommand());
  }

  public Command autoIntake() {
    return race(
        autoIntakeNoPivot(),
        pivot.getPivotCommand(() -> Rotation2d.fromDegrees(PivotIO.MIN_ANGLE)));
  }

  public Command autoIntakeNoPivot() {
    return parallel(intake.getIntakeCommand(), feeder.getFeedCommand())
        .until(noteDetector::hasNoteInShooter);
  }

  public Command getPivotManualControlCommand(DoubleSupplier pivotSupplier) {
    return pivot.getManualOverrideCommand(pivotSupplier);
  }

  public Command getPivotRestingPositionCommand() {
    return pivot.getRestingPositionCommand();
  }

  public Command getAutoAmpShotCommand() {
    return deadline(
        waitUntil(() -> flywheel.isUpToSpeed() && pivot.isAtSetpoint())
            .andThen(feeder.getFeedCommand().withTimeout(1)),
        // Gets canceled when the above finishes
        pivot.getPivotCommand(
            () -> {
              return Rotation2d.fromDegrees(PivotIO.MAX_ANGLE);
            }),
        flywheel.getAmpShotCommand());
    // return flywheel.getAmpShotCommand();
  }

  public void setPivotDefaultCommand(Command command) {
    pivot.setDefaultCommand(command);
  }

  public Command feed() {
    return feeder.getFeedCommand();
  }

  public Command shoot() {
    return deadline(
        waitUntil(flywheel::isUpToSpeed).andThen(feeder.getFeedCommand().withTimeout(1)),
        flywheel.getSpeakerShotCommand());
  }

  public Command shootDuringAuto() {
    return waitUntil(() -> flywheel.isUpToSpeed() && pivot.isAtSetpoint())
        .andThen(feeder.getFeedCommand().withTimeout(1));
  }

  public Command getFlywheelSpinUp() {
    return flywheel.getSpeakerShotCommand();
  }

  public Command getSourceIntakeCommand() {
    return parallel(
            flywheel.getSourceIntakeCommand(),
            feeder.getUnfeedCommand(),
            pivot.getPivotCommand(
                () -> {
                  return Rotation2d.fromDegrees(45);
                }))
        .until(noteDetector::hasNoteInShooter);
  }

  private double speakerY = 5.5;
  private double speakerZ = 1.5;
  private double speakerToRobotDistanceOffset = 0.254; // GOD FUCKING DAMNIT TOM

  public Command getAutoSpeakerShotCommand(Supplier<Translation2d> botTranslationSupplier) {
    // return none();
    return deadline(
        waitUntil(() -> flywheel.isUpToSpeed() && pivot.isAtSetpoint())
            .withTimeout(2)
            .andThen(feeder.getFeedCommand().withTimeout(1)),
        getAutoSpeakerAimCommand(botTranslationSupplier));
  }

  public Command getAutoSpeakerAimCommand(Supplier<Translation2d> botTranslationSupplier) {
    return pivot.getPivotCommand(
        () -> {
          return aimAtHeight(botTranslationSupplier.get(), speakerZ);
        });
  }

  public Command getTeleopAutoAimCommand(
      SwerveSubsystem drivebase, DoubleSupplier xVel, DoubleSupplier yVel) {
    return parallel(
        getAutoSpeakerShotCommand(() -> drivebase.getPose().getTranslation()),
        drivebase.getDriveCommand(
            xVel,
            yVel,
            () -> aimAtSpeaker(drivebase.getPose(), drivebase.getFieldRelativeVelocities())));
    // Michael was here));
  }

  public Command getSubwooferShotCommand() {
    return pivot.getPivotCommand(() -> Rotation2d.fromDegrees(49));
  }

  public Command getPodiumShotCommand() {
    return pivot.getPivotCommand(() -> Rotation2d.fromDegrees(32.1));
  }

  private Rotation2d aimAtHeight(Translation2d currentBotPosition, double height) {
    double speakerX = Robot.isOnRed() ? Field.FIELD_LENGTH.in(Meters) : 0;
    double targetDistance = currentBotPosition.getDistance(new Translation2d(speakerX, speakerY));
    targetDistance += speakerToRobotDistanceOffset;
    Logger.recordOutput("Shooter/autoAim/pivot/targetHeight", height);
    Logger.recordOutput("Shooter/autoAim/pivot/targetDistance", targetDistance);

    double targetPivotNoFudge = Math.atan(height / targetDistance);
    double fudgeFactor = Radians.convertFrom(fudgeFactorLerpTable.get(targetDistance), Degrees);
    Logger.recordOutput("Shooter/autoAim/pivot/targetPivotNoFudge", targetPivotNoFudge);
    Logger.recordOutput("Shooter/autoAim/pivot/pivotFudgeFactor", fudgeFactor);

    return new Rotation2d(targetPivotNoFudge + fudgeFactor);
  }

  // ROBOT ROTATE MATH
  private double aimAtSpeaker(Pose2d currentBotPose, ChassisSpeeds robotSpeeds) {
    Translation2d speakerPosition =
        new Translation2d(Robot.isOnRed() ? Field.FIELD_LENGTH.in(Meters) : 0, speakerY);
    Translation2d currentBotPosition = currentBotPose.getTranslation();

    // Calculate position and velocity of speaker relative to robot
    Translation2d speakerRelativePosition = speakerPosition.minus(currentBotPosition);
    double speakerRelativeVXMpS = -robotSpeeds.vxMetersPerSecond;
    double speakerRelativeVYMpS = -robotSpeeds.vyMetersPerSecond;
    Logger.recordOutput("Shooter/autoAim/yaw/speakerRelativeVXMps", speakerRelativeVXMpS);
    Logger.recordOutput("Shooter/autoAim/yaw/speakerRelativeVYMps", speakerRelativeVYMpS);

    double targetBotYawRad =
        Math.atan2(speakerRelativePosition.getY(), speakerRelativePosition.getX());
    Logger.recordOutput("Shooter/autoAim/yaw/targetBotYawRad", targetBotYawRad);

    double speakerRelativeVelocityDirectionRad =
        Math.atan2(speakerRelativeVYMpS, speakerRelativeVXMpS);
    double speakerRelativeVelocityMagnitudeMpS =
        Math.hypot(speakerRelativeVXMpS, speakerRelativeVYMpS);

    double speakerRelativeVParallel =
        Math.cos(speakerRelativeVelocityDirectionRad - targetBotYawRad)
            * speakerRelativeVelocityMagnitudeMpS;
    Logger.recordOutput(
        "Shooter/autoAim/yaw/speakerRelativeVParallelMpS", speakerRelativeVParallel);
    double speakerRelativeVPerpendicular =
        Math.sin(speakerRelativeVelocityDirectionRad - targetBotYawRad)
            * speakerRelativeVelocityMagnitudeMpS;
    Logger.recordOutput(
        "Shooter/autoAim/yaw/speakerRelativeVPerpendicular", speakerRelativeVPerpendicular);

    double feedforward = speakerRelativeVPerpendicular / speakerRelativePosition.getNorm();
    Logger.recordOutput("Shooter/autoAim/yaw/feedforward", feedforward);

    double feedback =
        5
            * speakerYawPidController.calculate(
                currentBotPose.getRotation().getRadians(), targetBotYawRad);
    Logger.recordOutput("Shooter/autoAim/yaw/feedback", feedback);

    return (feedforward + feedback) / SwerveSubsystem.MAX_ANGULAR_SPEED.in(RadiansPerSecond);
  }

  public Command getClimberManualControl(DoubleSupplier climberControl) {
    return climb.getClimberManualControlCommand(climberControl);
  }
}
