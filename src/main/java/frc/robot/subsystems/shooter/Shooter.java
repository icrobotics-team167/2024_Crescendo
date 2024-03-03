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
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.Robot;
import frc.robot.subsystems.misc.LightSubsystem;
import frc.robot.subsystems.misc.interfaceLayers.*;
import frc.robot.subsystems.misc.interfaceLayers.LightsIOBlinkin.Colors;
import frc.robot.subsystems.shooter.interfaceLayers.FeederIO;
import frc.robot.subsystems.shooter.interfaceLayers.FlywheelIO;
import frc.robot.subsystems.shooter.interfaceLayers.IntakeIO;
import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIO;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/** A class containing all the logic and commands to make the shooter mechanism work. */
public class Shooter {
  private final FlywheelSubsystem flywheel;
  private final PivotSubsystem pivot;
  private final NoteDetectorSubsystem noteDetector;
  private final IntakeSubsystem intake;
  private final FeederSubsystem feeder;
  private final LightSubsystem light;

  private PIDController autoAimController;

  public Shooter(
      FeederIO feederIO,
      FlywheelIO flywheelIO,
      PivotIO pivotIO,
      NoteDetectorIO noteDetectorIO,
      IntakeIO intakeIO,
      LightsIO lightIO) {
    flywheel = new FlywheelSubsystem(flywheelIO);
    pivot = new PivotSubsystem(pivotIO);
    noteDetector = new NoteDetectorSubsystem(noteDetectorIO);
    intake = new IntakeSubsystem(intakeIO);
    feeder = new FeederSubsystem(feederIO);
    light = new LightSubsystem(lightIO);

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
    return parallel(intake.getIntakeCommand(), feeder.getFeedCommand())
        .until(noteDetector::hasNote)
        .finallyDo(() -> light.setColor(Colors.GOLD));
  }

  public Command getManualControlCommand(DoubleSupplier pivotSupplier) {
    return pivot.getManualOverrideCommand(pivotSupplier);
  }

  public Command getPivotRestingPositionCommand() {
    return pivot.getRestingPositionCommand();
  }

  public Command getAutoAmpShotCommand() {
    return deadline(
            waitUntil(flywheel::isUpToSpeed).andThen(feeder.getFeedCommand().withTimeout(2)),
            parallel( // Gets canceled when the above finishes
                pivot.getPivotCommand(
                    () -> {
                      return Rotation2d.fromDegrees(90);
                    }),
                runOnce(() -> light.setColorValue(1705))),
            flywheel.getAmpShotCommand())
        .finallyDo(() -> light.setColor(Colors.GREEN));
    // return flywheel.getAmpShotCommand();
  }

  public void setPivotDefaultCommand(Command command) {
    pivot.setDefaultCommand(command);
  }

  public Command feed() {
    return feeder.getFeedCommand();
  }

  public Command shoot() {
    return parallel(
            deadline(
                waitUntil(flywheel::isUpToSpeed).andThen(feeder.getFeedCommand().withTimeout(1)),
                flywheel.getSpeakerShotCommand()),
            light.setColorValueCommand(1705))
        .finallyDo(() -> light.setColor(Colors.GREEN));
  }

  public Command getSourceIntakeCommand() {
    return parallel(
            flywheel.getSourceIntakeCommand(),
            feeder.getUnfeedCommand(),
            pivot.getPivotCommand(
                () -> {
                  return Rotation2d.fromDegrees(45);
                }))
        .until(noteDetector::hasNote);
  }

  private double speakerY = 5.5;
  private double speakerZ = 1.5;
  private double speakerToRobotDistanceOffset = 0.254; // GOD FUCKING DAMNIT TOM

  public Command getAutoSpeakerShotCommand(SwerveSubsystem drivebase) {
    // return none();
    return deadline(
            waitUntil(flywheel::isUpToSpeed).andThen(feeder.getFeedCommand().withTimeout(1)),
            runOnce(
                () ->
                    PPHolonomicDriveController.setRotationTargetOverride(
                        () ->
                            Optional.of(
                                aimAtPosition(
                                    drivebase,
                                    new Translation2d(
                                        Robot.isOnRed() ? Field.FIELD_LENGTH.in(Meters) : 0,
                                        speakerY))))),
            parallel(
                pivot.getPivotCommand(
                    () -> {
                      return aimAtHeight(drivebase, speakerZ);
                    }),
                flywheel.getSpeakerShotCommand(),
                runOnce(() -> light.setColorValue(1705))))
        .finallyDo(
            () -> {
              light.setColor(Colors.GREEN);
              PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty());
            });
  }

  public Command getTeleopAutoAimCommand(
      SwerveSubsystem drivebase, DoubleSupplier xVel, DoubleSupplier yVel) {
    return parallel(
        pivot.getPivotCommand(
            () -> {
              Rotation2d targetAngle = aimAtHeight(drivebase, speakerZ);
              if (Math.abs(pivot.getAngle().getDegrees() - targetAngle.getDegrees()) < 0.1) {
                light.setColorValue(1465);
              } else {
                light.setColor(Colors.GOLD);
              }
              return targetAngle;
            }),
        drivebase.getDriveCommand(
            xVel,
            yVel,
            () -> {
              return aimToYaw(
                  drivebase,
                  aimAtPosition(
                      drivebase,
                      new Translation2d(
                          Robot.isOnRed() ? Field.FIELD_LENGTH.in(Meters) : 0, speakerY)));
              // Michael was here
            }));
  }

  private Rotation2d aimAtHeight(SwerveSubsystem drivebase, double height) {
    double speakerX = Robot.isOnRed() ? Field.FIELD_LENGTH.in(Meters) : 0;
    Translation2d currentBotPosition = drivebase.getPose().getTranslation();
    double targetDistance = currentBotPosition.getDistance(new Translation2d(speakerX, speakerY));
    targetDistance += speakerToRobotDistanceOffset;
    // Proportional fudge factor
    // Close: ~1.75 meters, ~ 1 degree higher aim
    // Far: ~3 meters, no fudge
    double fudgeFactor =
        MathUtil.interpolate(-0.75, 0, MathUtil.clamp((targetDistance - 1.75) / (3 - 1.75), 0, 1));
    return new Rotation2d(
        Math.atan(height / targetDistance) + Radians.convertFrom(fudgeFactor, Degrees));
  }

  // ROBOT ROTATE MATH
  private Rotation2d aimAtPosition(SwerveSubsystem drivebase, Translation2d position) {
    Translation2d currentBotPosition = drivebase.getPose().getTranslation();
    Rotation2d targetBotYaw = position.minus(currentBotPosition).getAngle();
    return targetBotYaw;
  }

  private double aimToYaw(SwerveSubsystem drivebase, Rotation2d targetBotYaw) {
    Rotation2d currentBotYaw = drivebase.getPose().getRotation();
    return autoAimController.calculate(currentBotYaw.getDegrees(), targetBotYaw.getDegrees());
  }
}
