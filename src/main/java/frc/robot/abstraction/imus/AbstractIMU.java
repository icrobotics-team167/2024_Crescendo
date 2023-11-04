package frc.robot.abstraction.imus;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;

/**
 * The abstraction class for inertial measurement units.
 */
public abstract class AbstractIMU {
    /**
     * The maximum number of times that the motor will attempt to apply
     * configurations,
     * if it takes more than this amount of tries assume there's something wrong and
     * abort.
     */
    public final int maximumRetries = 5;

    /**
     * Reset IMU to factory default.
     */
    public abstract void factoryDefault();

    /**
     * Clears sticky faults on the IMU.
     */
    public abstract void clearStickyFaults();

    /**
     * Configures the IMU's mount pose.
     * 
     * @param yaw   Mount pose yaw.
     * @param pitch Mount pose pitch.
     * @param roll  Mount pose roll.
     */
    public abstract void configureMountPose(double yaw, double pitch, double roll);

    /**
     * Sets the offset that will be subtracted from the IMU's output.
     * 
     * @param offset The offset, as a Rotation3d object.
     */
    public abstract void setOffset(Rotation3d offset);

    /**
     * Gets the IMU's raw rotational output, without any offset.
     * 
     * @return The raw output, without any offset, as a Rotation3d object.
     */
    public abstract Rotation3d getRawRotation3d();

    /**
     * Gets the IMU's rotational output.
     * 
     * @return The output, as a Rotation3d object.
     */
    public abstract Rotation3d getRotation3d();

    /**
     * Gets the IMU's offset.
     * 
     * @return The offset.
     */
    public abstract Rotation3d getOffset();

    /**
     * If the IMU is capable of detecting acceleration, returns an Optional object
     * that contains a Translation3d representing the robot's acceleration.
     * Otherwise, returns a blank Optional object.
     * 
     * @return An Optional object that might contain a Translation3d.
     */
    public abstract Optional<Translation3d> getAccel();
}
