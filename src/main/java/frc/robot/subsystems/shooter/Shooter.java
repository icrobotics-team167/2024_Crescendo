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
import frc.robot.Constants.Driving;
import frc.robot.Constants.Field;
import frc.robot.Robot;
import frc.robot.subsystems.misc.LightSubsystem;
import frc.robot.subsystems.misc.interfaceLayers.*;
import frc.robot.subsystems.misc.interfaceLayers.LightsIOBlinkin.Colors;
import frc.robot.subsystems.shooter.interfaceLayers.ClimberIO;
import frc.robot.subsystems.shooter.interfaceLayers.FeederIO;
import frc.robot.subsystems.shooter.interfaceLayers.FlywheelIO;
import frc.robot.subsystems.shooter.interfaceLayers.IntakeIO;
import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIO;
import frc.robot.subsystems.shooter.interfaceLayers.PivotIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Optional;
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

  private PIDController autoAimController;

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

    autoAimController = new PIDController(0.06, 0, 0.001);
    autoAimController.enableContinuousInput(-180, 180);
  }

  public Command intake() {
    return intake.getIntakeCommand();
  }

  public Command intakeOut() {
    return parallel(
        intake.getIntakeOutCommand(), feeder.getUnfeedCommand(), flywheel.getSourceIntakeCommand());
  }

  public Command autoIntake() {
    return parallel(intake.getIntakeCommand(), feeder.getFeedCommand())
        .until(noteDetector::hasNote)
        .andThen(feeder.getUnfeedCommand().withTimeout(.5))
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

  public Command getAutoSpinUp() {
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
        .until(noteDetector::hasNote);
  }

  private double speakerY = 5.5;
  private double speakerZ = 1.5;
  private double speakerToRobotDistanceOffset = 0.254; // GOD FUCKING DAMNIT TOM

  public Command getAutoSpeakerShotCommand(Supplier<Translation2d> botTranslationSupplier) {
    // return none();
    return deadline(
            waitUntil(flywheel::isUpToSpeed).andThen(feeder.getFeedCommand().withTimeout(1)),
            runOnce(
                () ->
                    PPHolonomicDriveController.setRotationTargetOverride(
                        () ->
                            Optional.of(
                                aimAtPosition(
                                    botTranslationSupplier.get(),
                                    new Translation2d(
                                        Robot.isOnRed() ? Field.FIELD_LENGTH.in(Meters) : 0,
                                        speakerY))))),
            parallel(
                pivot.getPivotCommand(
                    () -> {
                      return aimAtHeight(botTranslationSupplier.get(), speakerZ);
                    }),
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
              Rotation2d targetAngle = aimAtHeight(drivebase.getPose().getTranslation(), speakerZ);
              if (Math.abs(pivot.getAngle().getDegrees() - targetAngle.getDegrees()) < 0.2) {
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
                      drivebase.getPose().getTranslation(),
                      new Translation2d(
                          Robot.isOnRed() ? Field.FIELD_LENGTH.in(Meters) : 0, speakerY)));
              // Michael was here
            }));
  }

  public Command getSubwooferShotCommand() {
    return pivot.getPivotCommand(
        () -> {
          Rotation2d targetAngle = Rotation2d.fromDegrees(48.4);
          if (Math.abs(pivot.getAngle().getDegrees() - targetAngle.getDegrees()) < 0.2) {
            light.setColorValue(1465);
          } else {
            light.setColor(Colors.GOLD);
          }
          return targetAngle;
        });
  }

  // 48.4 degrees for at subwoofer measured 47.3
  // 32 degrees for at podium
  private Rotation2d aimAtHeight(Translation2d currentBotPosition, double height) {
    double speakerX = Robot.isOnRed() ? Field.FIELD_LENGTH.in(Meters) : 0;
    double targetDistance = currentBotPosition.getDistance(new Translation2d(speakerX, speakerY));
    targetDistance += speakerToRobotDistanceOffset;
    // Proportional fudge factor
    // Close: ~1 meters, ~ 2.5 degree higher aim
    // Far: ~3 meters, ~ 0.75 degree lower aim
    double fudgeFactor = MathUtil.interpolate(2.5, 6.1, (targetDistance - 1) / (3 - 1));
    // lets hope this works. YOLO
    return new Rotation2d(
        Math.atan(height / targetDistance) + Radians.convertFrom(fudgeFactor, Degrees));
  }

  // ROBOT ROTATE MATH
  private Rotation2d aimAtPosition(Translation2d currentBotPosition, Translation2d targetPosition) {
    Rotation2d targetBotYaw = targetPosition.minus(currentBotPosition).getAngle();
    return targetBotYaw;
  }

  private double aimToYaw(SwerveSubsystem drivebase, Rotation2d targetBotYaw) {
    Rotation2d currentBotYaw = drivebase.getPose().getRotation();
    Logger.recordOutput("Shooter/autoAim/yawError", targetBotYaw.minus(currentBotYaw));
    double pidOut =
        autoAimController.calculate(currentBotYaw.getDegrees(), targetBotYaw.getDegrees())
            * (drivebase.isSlowmode() ? 1 : Driving.SLOWMODE_MULTIPLIER);
    Logger.recordOutput("Shooter/autoAim/pidOut", pidOut);
    return pidOut;
  }

  public Command getClimberManualControl(DoubleSupplier climberControl) {
    return climb.getClimberManualControlCommand(climberControl);
  }
}
