package frc.robot.abstraction.imus;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A CTRE Pigeon 2 Inertial Measurement Unit.
 */
public class Pigeon2IMU extends AbstractIMU {
    /**
     * The Pigeon 2 object.
     */
    WPI_Pigeon2 imu;

    /**
     * The output offset.
     */
    Rotation3d offset;

    /**
     * Configures a new Pigeon 2 IMU.
     * 
     * @param CANID The CAN ID of the Pigeon 2.
     */
    public Pigeon2IMU(int CANID) {
        imu = new WPI_Pigeon2(CANID);
    }

    @Override
    public void factoryDefault() {
        configurePigeon(() -> imu.configFactoryDefault());
        configurePigeon(() -> imu.configEnableCompass(false)); // Compass causes weird readings
    }

    @Override
    public void configureMountPose(double yaw, double pitch, double roll) {
        configurePigeon(() -> imu.configMountPose(yaw, pitch, roll));
    }

    @Override
    public void clearStickyFaults() {
        configurePigeon(() -> imu.clearStickyFaults());
    }

    @Override
    public void setOffset(Rotation3d offset) {
        this.offset = offset;
    }

    @Override
    public Rotation3d getRawRotation3d() {
        double[] wxyz = new double[4];
        imu.get6dQuaternion(wxyz);
        return new Rotation3d(new Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]));
    }

    @Override
    public Rotation3d getRotation3d() {
        return getRawRotation3d().minus(getRawRotation3d());
    }

    @Override
    public Rotation3d getOffset() {
        return offset;
    }

    @Override
    public Optional<Translation3d> getAccel() {
        short[] initial = new short[3];
        imu.getBiasedAccelerometer(initial);
        return Optional.of(new Translation3d(initial[0], initial[1], initial[2]).times(9.81 / 16384.0));
    }

    private void configurePigeon(Supplier<ErrorCode> config) {
        for (int i = 0; i < maximumRetries; i++) {
            if (config.get() == ErrorCode.OK) {
                return;
            }
        }
        DriverStation.reportWarning("Failed to configure Pigeon 2 IMU on CAN ID " + imu.getDeviceID(), true);
    }
}
