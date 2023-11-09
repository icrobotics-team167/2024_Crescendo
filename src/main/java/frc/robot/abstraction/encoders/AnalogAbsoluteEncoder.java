package frc.robot.abstraction.encoders;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class AnalogAbsoluteEncoder extends AbstractAbsoluteEncoder {

    int port;
    AnalogEncoder encoder;
    boolean inverted;

    /**
     * Constructs a new analog absolute encoder.
     * 
     * @param port The port number on the RoboRIO in which the encoder is connected
     *             to.
     */
    public AnalogAbsoluteEncoder(int port) {
        this(port, new Rotation2d());
    }

    public AnalogAbsoluteEncoder(int port, Rotation2d offset) {
        this.port = port;
        encoder = new AnalogEncoder(port);
        inverted = false;
        configureOffset(offset);
    }

    @Override
    public void factoryDefault() {
        // Do nothing
    }

    @Override
    public void clearStickyFaults() {
        // Do nothing
    }

    @Override
    public void configureInverted(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public void configureOffset(Rotation2d offset) {
        offset = Rotation2d.fromRotations(offset.getRotations() % 1);
        if (offset.getRotations() < 0) {
            offset = Rotation2d.fromRotations(1 - offset.getRotations());
        }
        encoder.setPositionOffset(offset.getRotations());
    }

    @Override
    public Rotation2d getAbsolutePosition() {
        return Rotation2d.fromDegrees((inverted ? -1 : 1) * (encoder.getAbsolutePosition() * 360 - 180 - encoder.getPositionOffset() ));
    }

    @Override
    public Object getAbsoluteEncoder() {
        return encoder;
    }

}
