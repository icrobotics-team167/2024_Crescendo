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

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.race;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.subsystems.misc.interfaceLayers.LightsIO;
import frc.robot.subsystems.misc.interfaceLayers.LightsIOBlinkin;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.interfaceLayers.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.interfaceLayers.*;
import frc.robot.util.MathUtils;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private LoggedDashboardChooser<Command> autoSelector;

  private final SwerveSubsystem drivebase;
  private final Shooter shooter;
  // private final LightSubsystem light;

  private CommandJoystick primaryLeftStick = new CommandJoystick(0);
  private CommandJoystick primaryRightStick = new CommandJoystick(1);
  private CommandJoystick secondaryLeftStick = new CommandJoystick(2);
  private CommandJoystick secondaryRightStick = new CommandJoystick(3);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Robot.currentMode) {
      case REAL:
        drivebase =
            new SwerveSubsystem(
                new GyroIOPigeon2(false),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        shooter =
            new Shooter(
                new FeederIOSparkFlex(),
                new FlywheelIOSparkFlex(),
                new PivotIOSparkFlex(),
                new NoteDetectorIOTimeOfFlight(),
                new IntakeIOTalonFX(),
                new LightsIOBlinkin(),
                new ClimberIO() {});
        // light = new LightSubsystem(new LightsIOBlinkin());
        break;
      case SIM:
        drivebase =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        shooter =
            new Shooter(
                new FeederIO() {},
                new FlywheelIO() {},
                new PivotIO() {},
                new NoteDetectorIO() {},
                new IntakeIO() {},
                new LightsIO() {},
                new ClimberIO() {});
        break;
      default:
        drivebase =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter =
            new Shooter(
                new FeederIO() {},
                new FlywheelIO() {},
                new PivotIO() {},
                new NoteDetectorIO() {},
                new IntakeIO() {},
                new LightsIO() {},
                new ClimberIO() {});
        // light = new LightSubsystem(new LightsIO() {});
    }
    NamedCommands.registerCommand(
        "Score in speaker",
        shooter.getAutoSpeakerShotCommand(() -> drivebase.getPose().getTranslation()));
    NamedCommands.registerCommand("Intake", shooter.autoIntake());
    NamedCommands.registerCommand("Intake Out", shooter.intakeOut());
    NamedCommands.registerCommand("Spin up flywheel", shooter.getFlywheelSpinUp());

    // Configure the trigger bindings
    configureBindings();
    // System.out.println("Deploy directory: " + Filesystem.getDeployDirectory());
    autoSelector = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    autoSelector.addOption(
        "The One Piece is real",
        race(shooter.getSubwooferShotCommand(), shooter.getFlywheelSpinUp()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    DoubleSupplier primaryLeftStickSide =
        () ->
            MathUtils.inOutDeadband(
                -primaryLeftStick.getX(),
                Driving.Deadbands.PRIMARY_LEFT_INNER,
                Driving.Deadbands.PRIMARY_LEFT_OUTER,
                Driving.PRIMARY_DRIVER_EXPONENT);
    DoubleSupplier primaryLeftStickForward =
        () ->
            MathUtils.inOutDeadband(
                -primaryLeftStick.getY(),
                Driving.Deadbands.PRIMARY_LEFT_INNER,
                Driving.Deadbands.PRIMARY_LEFT_OUTER,
                Driving.PRIMARY_DRIVER_EXPONENT);
    DoubleSupplier primaryRightStickSide =
        () ->
            MathUtils.inOutDeadband(
                -primaryRightStick.getX(),
                Driving.Deadbands.PRIMARY_RIGHT_INNER,
                Driving.Deadbands.PRIMARY_RIGHT_OUTER,
                Driving.PRIMARY_DRIVER_EXPONENT);
    DoubleSupplier secondaryLeftStickForwards =
        () ->
            MathUtils.inOutDeadband(
                secondaryLeftStick.getY(),
                Driving.Deadbands.SECONDARY_LEFT_INNER,
                Driving.Deadbands.SECONDARY_LEFT_OUTER,
                Driving.SECONDARY_DRIVER_EXPONENT);
    DoubleSupplier secondaryRightStickForwards =
        () ->
            MathUtils.inOutDeadband(
                secondaryRightStick.getY(),
                Driving.Deadbands.SECONDARY_LEFT_INNER,
                Driving.Deadbands.SECONDARY_LEFT_OUTER,
                Driving.SECONDARY_DRIVER_EXPONENT);
    drivebase.setDefaultCommand(
        drivebase.getDriveCommand(
            primaryLeftStickForward, primaryLeftStickSide, primaryRightStickSide));

    primaryLeftStick
        .trigger()
        .whileTrue(new StartEndCommand(drivebase::setSlowmode, drivebase::unsetSlowmode));
    primaryLeftStick.button(2).whileTrue(drivebase.getAmpAlign(primaryLeftStickSide));
    primaryLeftStick.button(3).onTrue(new InstantCommand(drivebase::resetGyroToForwards));

    primaryRightStick
        .trigger()
        .whileTrue(
            shooter.getTeleopAutoAimCommand(
                drivebase, primaryLeftStickForward, primaryLeftStickSide));
    primaryRightStick.button(2).onTrue(new InstantCommand(drivebase::stopWithX));
    primaryRightStick.button(4).whileTrue(shooter.getSubwooferShotCommand());
    primaryRightStick.button(5).whileTrue(shooter.getPodiumShotCommand());

    secondaryLeftStick
        .trigger()
        .whileTrue(shooter.getManualControlCommand(secondaryLeftStickForwards));
    secondaryLeftStick.button(2).whileTrue(shooter.intakeOut());
    // shooter.setPivotDefaultCommand(shooter.getPivotRestingPositionCommand());
    secondaryLeftStick.button(3).whileTrue(shooter.getAutoAmpShotCommand());
    secondaryLeftStick.button(4).whileTrue(shooter.getSourceIntakeCommand());

    secondaryRightStick.trigger().whileTrue(shooter.autoIntake());
    secondaryRightStick.button(2).whileTrue(shooter.shoot());
    secondaryRightStick.button(3).whileTrue(shooter.getFlywheelSpinUp());
    secondaryRightStick.button(4).whileTrue(shooter.feed());

    secondaryRightStick
        .button(10)
        .whileTrue(shooter.getClimberManualControl(secondaryRightStickForwards));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.get();
  }

  public void teleopInit() {
    drivebase.resetGyroFromPose();
  }
}
