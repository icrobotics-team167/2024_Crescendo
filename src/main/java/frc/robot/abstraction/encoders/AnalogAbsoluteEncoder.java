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
        double offsetValue = offset.getRotations();
        offsetValue = offsetValue % 1;
        if (offsetValue < 0) {
            offsetValue = 1 - offsetValue;
        }
        encoder.setPositionOffset(offset.getRotations());
    }

    @Override
    public Rotation2d getOffset() {
        return Rotation2d.fromRotations(encoder.getPositionOffset());
    }

    @Override
    public Rotation2d getAbsolutePosition() {
        return Rotation2d.fromDegrees(
                (inverted ? -1 : 1) * ((encoder.getAbsolutePosition() - encoder.getPositionOffset()) * 360 - 180));
    }

    @Override
    public Object getAbsoluteEncoder() {
        return encoder;
    }

}
