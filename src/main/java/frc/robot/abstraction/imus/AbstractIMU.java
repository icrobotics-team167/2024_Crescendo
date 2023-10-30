package frc.robot.abstraction.imus;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

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

    public abstract void clearStickyFaults();

    public abstract void configureMountPose(double pitch, double roll, double yaw);

    public abstract void setOffset(Rotation3d offset);

    public abstract Rotation3d getRawRotation3d();

    public abstract Rotation3d getRotation3d();

    public abstract Rotation3d getOffset();

    public abstract Optional<Translation3d> getAccel();
}
