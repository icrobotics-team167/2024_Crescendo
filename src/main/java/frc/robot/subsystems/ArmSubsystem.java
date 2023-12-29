package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.abstraction.motors.RevNEO500;
import frc.robot.abstraction.motors.RevNEO550;
import frc.robot.arm.*;
import frc.robot.Constants.Robot.Arm;

/**
 * A SubsystemBase class to implement the arm.
 */
public class ArmSubsystem extends SubsystemBase {
    /**
     * The claw mechanism.
     */
    private final Claw claw;
    /**
     * The extension mechanism.
     */
    private final Extension extension;
    /**
     * The pivot mechanism.
     */
    private final Pivot pivot;

    /**
     * Constructs a new ArmSubsystem object.
     */
    public ArmSubsystem() {
        claw = new Claw(new RevNEO550(Arm.IDs.CLAW_MOTOR));
        extension = new Extension(new RevNEO550(Arm.IDs.EXTENSION_MOTOR),
                new DigitalInput(Arm.IDs.MIN_EXTENSION_SENSOR));
        pivot = new Pivot(new RevNEO500(Arm.IDs.PIVOT_LEADER), new RevNEO500(Arm.IDs.PIVOT_FOLLOWER));
    }

    /**
     * Moves the pivot mechanism.
     * 
     * @param speed The speed in which to move the pivot. 1.0 is pivot down full
     *              speed, -1.0 is pivot up full speed.
     */
    public void pivot(double speed) {
        pivot.move(speed);
    }

    /**
     * Moves the extension mechanism.
     * 
     * @param speed The speed in which to move the extension. 1.0 is extend out full
     *              speed, -1.0 is retract full speed.
     */
    public void extension(double speed) {
        extension.move(speed);
    }

    /**
     * Moves the arm.
     * 
     * @param pivotSpeed     The speed in which to move the pivot. 1.0 is pivot down full
     *                  speed, -1.0 is retract full speed.
     * @param extensionSpeed The speed in which to move the extension. 1.0 is extend out
     *                  full speed, -1.0 is retract full speed.
     */
    public void move(double pivotSpeed, double extensionSpeed) {
        pivot(pivotSpeed);
        extension(extensionSpeed);
    }

    /**
     * Runs the intake mechanism.
     */
    public void intake() {
        claw.move(Arm.Claw.INTAKE_SPEED);
    }

    /**
     * Runs the intake mechanism backwards to spit anything out.
     */
    public void outtake() {
        claw.move(-Arm.Claw.OUTTAKE_SPEED);
    }

    /**
     * Stops the intake mechanism.
     * 
     * <p>This method MUST be called if you want the intake mechanism to stop running, as otherwise it just runs forever.
     */
    public void stopIntake() {
        claw.move(0);
    }

    /**
     * Stops the arm.
     */
    public void stop() {
        move(0, 0);
    }

    /**
     * Gets the position of the arm.
     * @return The position, represented as an ArmPosition object.
     */
    public ArmPosition getPosition() {
        return new ArmPosition(pivot.getPosition(), extension.getPosition());
    }

    /**
     * Gets if the arm is too far up.
     * 
     * @return If the arm is above its max point, configured in Constants.
     */
    public boolean isTooFarUp() {
        return pivot.isTooFarUp();
    }

    /**
     * Gets if the arm is too far down.
     * 
     * @return If the arm is below its min point, configured in Constants.
     */
    public boolean isTooFarDown() {
        return pivot.isTooFarDown();
    }

    /**
     * Gets if the arm is too far in.
     * 
     * @return If the arm is retracted past its min point, configured in Constants.
     */
    public boolean isTooFarIn() {
        return extension.isTooFarIn();
    }

    /**
     * Gets if the arm is too far out.
     * 
     * @return If the arm is extended past its max point, configured in Constants.
     */
    public boolean isTooFarOut() {
        return extension.isTooFarOut();
    }
}
