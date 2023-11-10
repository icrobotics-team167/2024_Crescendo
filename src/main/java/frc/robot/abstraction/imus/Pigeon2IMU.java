package frc.robot.abstraction.imus;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/**
 * A CTRE Pigeon 2 Inertial Measurement Unit.
 */
public class Pigeon2IMU extends AbstractIMU {
    /**
     * The Pigeon 2 object.
     */
    Pigeon2 imu;
    Pigeon2Configuration configuration = new Pigeon2Configuration();
    Pigeon2Configurator configurator;

    /**
     * The output offset.
     */
    Rotation3d offset = new Rotation3d();

    /**
     * Configures a new Pigeon 2 IMU.
     * 
     * @param CANID The CAN ID of the Pigeon 2.
     */
    public Pigeon2IMU(int CANID) {
        imu = new Pigeon2(CANID);
        configurator = imu.getConfigurator();
    }

    @Override
    public void factoryDefault() {
        configuration = new Pigeon2Configuration();
        configuration.Pigeon2Features.EnableCompass = false;
        applyConfigs();
    }

    @Override
    public void configureMountPose(double yaw, double pitch, double roll) {
        configuration.MountPose.MountPoseYaw = yaw;
        configuration.MountPose.MountPosePitch = pitch;
        configuration.MountPose.MountPoseRoll = roll;
        applyConfigs();
    }

    @Override
    public void clearStickyFaults() {
        imu.clearStickyFaults();
    }

    @Override
    public void setOffset(Rotation3d offset) {
        this.offset = offset;
    }

    @Override
    public Rotation3d getRawRotation3d() {
        return new Rotation3d(new Quaternion(imu.getQuatW().getValue(), imu.getQuatX().getValue(),
                imu.getQuatY().getValue(), imu.getQuatZ().getValue()));
    }

    @Override
    public Rotation3d getRotation3d() {
        return getRawRotation3d().minus(getOffset());
    }

    @Override
    public Rotation3d getOffset() {
        return offset;
    }

    @Override
    public Optional<Translation3d> getAccel() {
        short[] initial = new short[3];
        // imu.getBiasedAccelerometer(initial);

        return Optional.of(new Translation3d(initial[0], initial[1], initial[2]).times(9.81 / 16384.0));
    }

    private void applyConfigs() {
        for (int i = 0; i < maximumRetries; i++) {
            if (configurator.apply(configuration) == StatusCode.OK) {
                return;
            }
        }
        DriverStation.reportWarning("Failed to configure Pigeon 2 IMU on CAN ID " + imu.getDeviceID(), true);
    }
}
