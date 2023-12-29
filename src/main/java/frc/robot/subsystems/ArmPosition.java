package frc.robot.subsystems;

/**
 * A record class to represent the arm mechanism's position. Contains preset
 * positions as static constants.
 * 
 * @param pivotAngle      The pivot angle, in degrees.
 * @param extensionLength The extension length, in inches.
 */
public record ArmPosition(double pivotAngle, double extensionLength) {
    /**
     * Override the preset's pivot angle.
     * 
     * @param newPivotAngle The new angle to override to.
     * @return The arm position with the angle overridden.
     */
    public ArmPosition withPivot(double newPivotAngle) {
        return new ArmPosition(newPivotAngle, this.extensionLength);
    }

    /**
     * Override the preset's extension length.
     * 
     * @param newExtensionLength The new length to override to.
     * @return The arm position with the length overridden.
     */
    public ArmPosition withExtension(double newExtensionLength) {
        return new ArmPosition(this.pivotAngle, newExtensionLength);
    }

    /**
     * The initial arm position.
     * 
     * <p>
     * The arm should be in this position at robot boot or else everything goes to
     * wack (we don't have absolute encoders on the arm)
     */
    public static final ArmPosition INITIAL = new ArmPosition(60, 0);
    /**
     * The arm position to intake a game piece from the ground.
     */
    public static final ArmPosition GROUND_INTAKE = new ArmPosition(-33, 0);
}
