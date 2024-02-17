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

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.interfaceLayers.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.interfaceLayers.*;
import frc.robot.util.MathUtils;
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
                new GyroIOPigeon2(),
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
                new IntakeIOSparkMax());
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
                new IntakeIO() {});
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
                new IntakeIO() {});
    }
    NamedCommands.registerCommand("Score in speaker", none()); // TODO: Implement
    NamedCommands.registerCommand("Intake", none());

    // Configure the trigger bindings
    configureBindings();
    autoSelector = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
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
    drivebase.setDefaultCommand(
        drivebase.getDriveCommand(
            () ->
                MathUtils.inOutDeadband(
                    -primaryLeftStick.getY(),
                    Driving.Deadbands.PRIMARY_LEFT_INNER,
                    Driving.Deadbands.PRIMARY_LEFT_OUTER,
                    Driving.PRIMARY_DRIVER_EXPONENT),
            () ->
                MathUtils.inOutDeadband(
                    -primaryLeftStick.getX(),
                    Driving.Deadbands.PRIMARY_LEFT_INNER,
                    Driving.Deadbands.PRIMARY_LEFT_OUTER,
                    Driving.PRIMARY_DRIVER_EXPONENT),
            () ->
                MathUtils.inOutDeadband(
                    -primaryRightStick.getX(),
                    Driving.Deadbands.PRIMARY_RIGHT_INNER,
                    Driving.Deadbands.PRIMARY_RIGHT_OUTER,
                    Driving.PRIMARY_DRIVER_EXPONENT)));

    primaryLeftStick
        .trigger()
        .whileTrue(new StartEndCommand(drivebase::setSlowmode, drivebase::unsetSlowmode));
    primaryRightStick.trigger().onTrue(new InstantCommand(drivebase::stopWithX));
    // primaryLeftStick.button(1).whileTrue(drivebase.getSysIDURCL());
    primaryLeftStick.button(1).whileTrue(drivebase.getAzimuthSysIDURCL());

    secondaryRightStick.trigger().whileTrue(shooter.autoIntake());
    secondaryLeftStick
        .trigger()
        .whileTrue(
            shooter.getManualControlCommand(
                () ->
                    MathUtils.inOutDeadband(
                        -secondaryLeftStick.getY(),
                        Driving.Deadbands.SECONDARY_LEFT_INNER,
                        Driving.Deadbands.SECONDARY_LEFT_OUTER,
                        Driving.SECONDARY_DRIVER_EXPONENT)));
    shooter.setPivotDefaultCommand(shooter.getPivotRestingPositionCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.get();
  }
}
