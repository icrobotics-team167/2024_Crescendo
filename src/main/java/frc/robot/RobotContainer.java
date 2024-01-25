// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.FieldRelativeDrive;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOTalonFX;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private LoggedDashboardChooser<Command> autoSelector;

  private SwerveSubsystem drivebase;

  private CommandJoystick primaryLeftStick = new CommandJoystick(0);
  private CommandJoystick primaryRightStick = new CommandJoystick(1);
  private CommandJoystick secondaryLeftStick = new CommandJoystick(2);
  private CommandJoystick secondaryRightStick = new CommandJoystick(3);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Robot.currentMode) {
      case REAL:
      case REPLAY:
        drivebase = new SwerveSubsystem(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));
        break;
      case SIM:
        drivebase = new SwerveSubsystem(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
        break;
      default:
        drivebase = new SwerveSubsystem(new GyroIO() {
        }, new ModuleIO() {
        }, new ModuleIO() {
        }, new ModuleIO() {
        }, new ModuleIO() {
        });
    }
    // Configure the trigger bindings
    configureBindings();
    autoSelector = new LoggedDashboardChooser<>("", AutoBuilder.buildAutoChooser());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivebase.setDefaultCommand(new FieldRelativeDrive(
        drivebase,
        () -> primaryLeftStick.getX(),
        () -> primaryLeftStick.getY(),
        () -> primaryRightStick.getX()));

    primaryLeftStick.trigger().whileTrue(new StartEndCommand(drivebase::setSlowmode, drivebase::unsetSlowmode));
    primaryRightStick.trigger().onTrue(new InstantCommand(drivebase::stopWithX));
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
