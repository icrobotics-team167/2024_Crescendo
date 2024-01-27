package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.struct.SwerveModuleStateStruct;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.abstraction.encoders.AbstractAbsoluteEncoder;
import frc.robot.abstraction.motors.AbstractMotor;
import frc.robot.Constants.Robot.SwerveDrive;
import frc.robot.Constants.Robot.SwerveDrive.Modules;
import frc.robot.helpers.Telemetry;
import frc.robot.helpers.Telemetry.Verbosity;

/**
 * A class representing a swerve module.
 */
public class Module {
  /**
   * The absolute max move speed that the modules can attain, in m/s.
   */
  private final double MAX_MOVE_SPEED;
  /**
   * The module ID for kinematics.
   */
  public final int moduleNumber;

  /**
   * The drive motor.
   */
  private AbstractMotor driveMotor;
  /**
   * The turn motor.
   */
  private AbstractMotor turnMotor;
  /**
   * The turn encoder.
   */
  private AbstractAbsoluteEncoder turnEncoder;

  /**
   * The desired state of the module.
   */
  private SwerveModuleState desiredState;

  /**
   * Constructs a new swerve module and initializes the motors and encoders.
   * 
   * @param moduleNumber  Module ID for kinematics.
   * @param driveMotor    Drive motor.
   * @param turnMotor     Turn motor.
   * @param turnEncoder   Turn encoder.
   * @param encoderOffset Encoder offset.
   */
  public Module(int moduleNumber, AbstractMotor driveMotor, AbstractMotor turnMotor,
      AbstractAbsoluteEncoder turnEncoder) {
    this.driveMotor = driveMotor;
    this.turnMotor = turnMotor;
    this.turnEncoder = turnEncoder;

    // Create max move speed constant for math
    MAX_MOVE_SPEED = getMetersPerRotation() * this.driveMotor.getMaxRPM() / 60.0;
    System.out.println(moduleName() + " max move speed: " + MAX_MOVE_SPEED);
    // Set module number (See moduleName() method for what values correspond to what
    // moduke)
    this.moduleNumber = moduleNumber;

    configure();
  }

  /**
   * Configures the motors.
   */
  private void configure() {
    // Configure drive motor
    driveMotor.clearStickyFaults();
    driveMotor.configureCurrentLimits(
        driveMotor.getNominalVoltage(),
        driveMotor.getPrimaryCurrentLimit(),
        driveMotor.getSecondaryCurrentLimit());
    driveMotor.configureMotorBrake(true);
    driveMotor.configureRampRate(SwerveDrive.ZERO_TO_FULL_TIME);
    driveMotor.configurePID(Modules.ControlParams.DRIVE_P, Modules.ControlParams.DRIVE_I,
        Modules.ControlParams.DRIVE_D);
    driveMotor.configureFeedForward(0, driveMotor.getNominalVoltage() / MAX_MOVE_SPEED, 0); // TODO: Tune
    driveMotor.configureEncoder(getMetersPerRotation());

    // Configure turn motor
    turnMotor.clearStickyFaults();
    turnMotor.configureCurrentLimits(
        turnMotor.getNominalVoltage(),
        turnMotor.getPrimaryCurrentLimit(),
        turnMotor.getSecondaryCurrentLimit());
    turnMotor.configureMotorBrake(false);
    turnMotor.configurePIDWrapping(true);
    turnMotor.configurePID(Modules.ControlParams.TURN_P, Modules.ControlParams.TURN_I, Modules.ControlParams.TURN_D);
    turnMotor.configureEncoder(360 / Modules.TURN_GEAR_RATIO);
    turnMotor.configureAbsoluteEncoder(turnEncoder, 360);
    turnMotor.configureInverted(true);
    turnMotor.setPosition(turnEncoder.getAbsolutePosition().getDegrees());
  }

  /**
   * Sets the desired state of the swerve module.
   * 
   * @param desiredState The desired state.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // It might be the case that flipping the desired angle and reversing driving
    // direction might be faster to move to, so check for that
    desiredState = SwerveModuleState.optimize(desiredState, getRotation());

    setRawState(desiredState);
  }

  public void setRawState(SwerveModuleState rawState) {
    this.desiredState = rawState;
    driveMotor.setVelocityReference(rawState.speedMetersPerSecond);
    // 0);
    //turnMotor.setPosition(turnEncoder.getAbsolutePosition().getDegrees());
    turnMotor.setTurnReference(rawState.angle);
  }

  /**
   * Sets whether the drive wheel is in brake mode or not.
   * 
   * @param brake True for brake, false for coast.
   */
  public void setWheelBrake(boolean brake) {
    driveMotor.configureMotorBrake(brake);
  }

  /**
   * Sets the desired angle of the swerve module, with no driving.
   * 
   * @param angle The desired angle.
   */
  public void setAngle(Rotation2d angle) {
    setDesiredState(new SwerveModuleState(0, angle));
  }

  /**
   * Gets the position of the swerve module.
   * 
   * @return A SwerveModuleState object representing the current state.
   */
  public SwerveModuleState getActualState() {
    return new SwerveModuleState(driveMotor.getVelocity(), getRotation());
  }

  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  /**
   * Gets the position for the swerve module for odometry.
   * 
   * @return A SwerveModulePosition object representing the current position.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotor.getPosition(), getRotation());
  }

  /**
   * Gets the rotation of the swerve module.
   * 
   * @return A Rotation2d object representing the current rotation.
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(turnMotor.getPosition());
  }

  /**
   * Gets the absolute max velocity of the module.
   * 
   * @return Max velocity, in meters per second.
   */
  public double getMaxVel() {
    return MAX_MOVE_SPEED;
  }

  /**
   * Resets the drive motor's positon.
   */
  public void resetPosition() {
    driveMotor.setPosition(0);
  }

  /**
   * Gets the meters driven by the module per rotation of the drive motor.
   * 
   * @return Meters per rotation.
   */
  public double getMetersPerRotation() {
    return Modules.WHEEL_CIRCUMFERENCE / Modules.DRIVE_GEAR_RATIO;
  }

  /**
   * Gets the name of the module for cleaner telemetry.
   * 
   * @return The module name.
   */
  private String moduleName() {
    switch (moduleNumber) {
      case 0:
        return "Front Left";
      case 1:
        return "Front Right";
      case 2:
        return "Back Left";
      case 3:
        return "Back Right";
      default:
        DriverStation.reportError("ERROR: Expected 4 swerve modules, but got 5.", false);
        return String.valueOf(moduleNumber);
    }
  }
}
