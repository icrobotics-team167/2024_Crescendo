package frc.robot.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.abstraction.encoders.AbstractAbsoluteEncoder;
import frc.robot.abstraction.motors.AbstractMotor;
import frc.robot.Constants.Robot.SwerveDrive;
import frc.robot.Constants.Robot.SwerveDrive.Modules;
import frc.robot.helpers.Telemetry;
import frc.robot.helpers.Telemetry.Verbosity;

public class Module {
  public int moduleNumber;
  private AbstractMotor driveMotor;
  private AbstractMotor turnMotor;
  private AbstractAbsoluteEncoder turnEncoder;
  private final double MAX_MOVE_SPEED;

  private SimpleMotorFeedforward driveMotorFF;

  private SwerveModuleState previousState = new SwerveModuleState();

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
    this.moduleNumber = moduleNumber;
    this.driveMotor = driveMotor;
    this.turnMotor = turnMotor;
    this.turnEncoder = turnEncoder;

    // Create max move speed constant for math
    MAX_MOVE_SPEED = getMetersPerRotation() * driveMotor.getMaxRPM() / 60.0;

    // Create feedforward for driving calculations
    driveMotorFF = createDriveFeedforward();

    // Configure drive motor.
    driveMotor.configureMotorBrake(true);
    driveMotor.configureCurrentLimits(
        driveMotor.getNominalVoltage(),
        driveMotor.getPrimaryCurrentLimit(),
        driveMotor.getSecondaryCurrentLimit());
    driveMotor.configureIntegratedEncoder(getMetersPerRotation());
    driveMotor.configrePID(0.25, 0, 0);
    driveMotor.configureRampRate(SwerveDrive.ZERO_TO_FULL_TIME);

    // Configure turn motor.
    turnMotor.configureMotorBrake(false);
    turnMotor.configureCurrentLimits(
        turnMotor.getNominalVoltage(),
        turnMotor.getPrimaryCurrentLimit(),
        turnMotor.getSecondaryCurrentLimit());
    turnMotor.configrePID(0.25, 0, 0.1);
    turnMotor.configurePIDWrapping(-180, 180);
    turnMotor.configureAbsoluteEncoder(turnEncoder);
  }

  /**
   * Sets the desired state of the swerve module.
   * 
   * @param desiredState The desired state.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // It might be faster to flip the desired rotation 180 degrees and reverse the
    // desired movement, so if that is the case, optimize the desired state.
    desiredState = SwerveModuleState.optimize(desiredState, turnEncoder.getAbsolutePosition());

    // If the desired drive speed is the same as the previous desired drive speed,
    // no need to do anything.
    if (desiredState.speedMetersPerSecond != previousState.speedMetersPerSecond) {
      double feedForward = driveMotorFF.calculate(desiredState.speedMetersPerSecond);
      driveMotor.setDriveReference(desiredState.speedMetersPerSecond, feedForward);
      Telemetry.sendNumber(moduleName() + " desired drive speed", desiredState.speedMetersPerSecond, Verbosity.HIGH);
      Telemetry.sendNumber(moduleName() + " drive feedforward", feedForward, Verbosity.HIGH);
      Telemetry.sendNumber(moduleName() + " actual drive speed", driveMotor.getVelocity(), Verbosity.HIGH);
    }

    // If the desired turn angle is the same as the previous desired turn angle, no
    // need to do anything.
    if (desiredState.angle != previousState.angle) {
      turnMotor.setTurnReference(desiredState.angle);
    }

    // Set previous desired angle for the next time this is run.
    previousState = desiredState;
  }

  /**
   * Sets the desired angle of the swerve module.
   * 
   * @param angle The desired angle.
   */
  public void setAngle(Rotation2d angle) {
    turnMotor.setTurnReference(angle);
    previousState.angle = angle;
  }

  /**
   * Gets the position of the swerve module.
   * 
   * @return A SwerveModuleState object representing the current state.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getVelocity(), turnEncoder.getAbsolutePosition());
  }

  /**
   * Gets the position for the swerve module for odometry.
   * 
   * @return A SwerveModulePosition object representing the current position.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotor.getPosition(), turnEncoder.getAbsolutePosition());
  }

  /**
   * Gets the rotation of the swerve module.
   * 
   * @return A Rotation2d object representing the current rotation.
   */
  public Rotation2d getRotation() {
    return turnEncoder.getAbsolutePosition();
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
    return Modules.WHEEL_CIRCUMFERENCE / Modules.GEAR_RATIO;
  }

  /**
   * Create the drive feedforward for swerve modules.
   *
   * @return Drive feedforward for drive motor on a swerve module.
   */
  private SimpleMotorFeedforward createDriveFeedforward() {
    double kv = driveMotor.getNominalVoltage() / MAX_MOVE_SPEED;
    /// ^ Volt-seconds per meter (max voltage divided by max acceleration)
    double ka = driveMotor.getNominalVoltage() / (SwerveDrive.MAX_ACCELERATION);
    /// ^ Volt-seconds^2 per meter (max voltage divided by max accel)
    return new SimpleMotorFeedforward(0, kv, ka);
  }

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
