// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands; // Keep this around so that we don't have to reimport when we add named commands
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

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

  public SendableChooser<Command> autoSelector = new SendableChooser<Command>();

  private final SwerveSubsystem driveBase = new SwerveSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  CommandJoystick primaryLeftStick = new CommandJoystick(Constants.Driving.Controllers.IDs.PRIMARY_LEFT);
  CommandJoystick primaryRightStick = new CommandJoystick(Constants.Driving.Controllers.IDs.PRIMARY_RIGHT);
  CommandJoystick secondaryLeftStick = new CommandJoystick(Constants.Driving.Controllers.IDs.SECONDARY_LEFT);
  CommandJoystick secondaryRightStick = new CommandJoystick(Constants.Driving.Controllers.IDs.SECONDARY_RIGHT);

  AimAtSpeaker aimAtSpeakerCommand;
  AimManualOverride aimManualOverrideCommand;
  AbsoluteFieldDrive driveControllerCommand;
  Intake intakeCommand;
  TestShooter testShooterCommand;

  private Timer disabledTimer = new Timer();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Create commands
    aimAtSpeakerCommand = new AimAtSpeaker(shooter, driveBase::overrideRotation, driveBase::getPose);
    aimManualOverrideCommand = new AimManualOverride(() -> MathUtil
        .applyDeadband(secondaryRightStick.getY(), Constants.Driving.Controllers.Deadbands.SECONDARY_RIGHT),
        secondaryRightStick.button(2), shooter);
    intakeCommand = new Intake(shooter);
    // driveControllerCommand = new AbsoluteFieldDrive(
    // driveBase,
    // () -> MathUtil.applyDeadband(-primaryLeftStick.getY(),
    // Constants.Driving.Controllers.Deadbands.PRIMARY_LEFT),
    // () -> MathUtil.applyDeadband(-primaryLeftStick.getX(),
    // Constants.Driving.Controllers.Deadbands.PRIMARY_LEFT),
    // () -> MathUtil.applyDeadband(-primaryRightStick.getX(),
    // Constants.Driving.Controllers.Deadbands.PRIMARY_RIGHT));

    // this is a system to test the auto tracking of target by removing control from
    // driver, it is a toggled switch. TODO: Make a better toggle function for
    // buttons that doesn't force use of commands.
    driveControllerCommand = new AbsoluteFieldDrive(
        driveBase,
        () -> MathUtil.applyDeadband(-primaryLeftStick.getY(), Constants.Driving.Controllers.Deadbands.PRIMARY_LEFT),
        () -> MathUtil.applyDeadband(-primaryLeftStick.getX(), Constants.Driving.Controllers.Deadbands.PRIMARY_LEFT),
        () -> MathUtil.applyDeadband(-primaryRightStick.getX(), Constants.Driving.Controllers.Deadbands.PRIMARY_RIGHT));

    // Register auto commands for PathPlanner
    NamedCommands.registerCommand("Intake", intakeCommand);
    NamedCommands.registerCommand("Aim At Speaker", aimAtSpeakerCommand);

    // Auto selector configuring
    autoSelector = AutoBuilder.buildAutoChooser(); // Load all the pathplanner autos
    // Load non-pathplanner autos
    autoSelector.addOption("Test Auto (Module Actuation)", new TestWheels(driveBase));
    SmartDashboard.putData(autoSelector);

    // Configure the trigger bindings
    configureBindings();

    // Set default commands
    driveBase.setDefaultCommand(driveControllerCommand);
    shooter.setDefaultCommand(aimManualOverrideCommand);
    // shooter.setDefaultCommand(new TestShooter(shooter));
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
    primaryLeftStick.trigger() // Trigger on the primary driver's left stick
        .whileTrue(new StartEndCommand(driveBase::setSlowMode, driveBase::unsetSlowMode)); // Press and hold for slow
                                                                                           // mode
    primaryRightStick.trigger() // Trigger on the primary driver's right stick
        .whileTrue(new StartEndCommand(driveBase::lockMotion, driveBase::unlockMotion)); // Press and hold to lock
                                                                                         // the drivebase
    primaryRightStick.button(2) // Button #2 on the primary driver's right stick
        .onTrue(new InstantCommand(driveBase::resetRotation)); // Resets which way the robot thinks is forward, used
                                                               // when the robot wasn't facing away from the driver
                                                               // station on boot
    secondaryRightStick.button(2) // Button #2 on the secondary driver's right stick
        .whileTrue(intakeCommand); // Intake a note while the button is held down. Automatically stops once a note
                                   // is loaded.
    secondaryRightStick.trigger() // Trigger on the secondary driver's right stick
        .whileTrue(aimAtSpeakerCommand); // Aim and shoot the note at the speaker.

    // primaryLeftStick.button(3).whileTrue(new StartEndCommand(driveBase::trackTag,
    // driveBase::setSlowMode));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  /**
   * Runs once at the start of autonomous.
   */
  public void autonomousInit() {
    driveBase.unlockMotion();
  }

  /**
   * Runs every robot tick during autonomous.
   */
  public void autonomousPeriodic() {
  }

  /**
   * Runs once at the start of teleop.
   */
  public void teleopInit() {
    driveBase.unlockMotion();
  }

  /**
   * Runs every robot tick during teleop.
   */
  public void teleopPeriodic() {

  }

  /**
   * Runs once at the end of an match.
   */
  public void endOfMatchInit() {
    driveBase.lockMotion();
    driveBase.setWheelBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  /**
   * Runs every robot tick after a match.
   */
  public void endOfMatchPeriodic() {
    if (disabledTimer.hasElapsed(Constants.END_OF_MATCH_LOCK)) {
      driveBase.unlockMotion();
      driveBase.setWheelBrake(false);
    }
  }

  /**
   * Runs once at robot boot, before a match.
   */
  public void preMatch() {
    driveBase.setWheelBrake(true);
    driveBase.setWheelsForward();
  }

  /**
   * Gets which alliance the robot is on
   * 
   * @return If the robot is on red alliance. False if it is on the blue alliance,
   *         or if the alliance cannot be loaded.
   */
  public static boolean isRedAlliance() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get() == Alliance.Red;
    }
    return false;
  }
}
