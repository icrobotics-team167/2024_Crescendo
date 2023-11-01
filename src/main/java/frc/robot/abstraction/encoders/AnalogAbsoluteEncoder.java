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
        this.port = port;
        encoder = new AnalogEncoder(port);
        inverted = false;
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
    public void configure(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public Rotation2d getAbsolutePosition() {
        return Rotation2d.fromDegrees((inverted ? -1 : 1) * (encoder.getAbsolutePosition() * 360 - 180));
    }

    @Override
    public Object getAbsoluteEncoder() {
        return encoder;
    }

}
