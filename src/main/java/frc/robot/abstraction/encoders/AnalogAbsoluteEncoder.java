package frc.robot.abstraction.encoders;

import edu.wpi.first.wpilibj.AnalogEncoder;

public class AnalogAbsoluteEncoder extends AbstractAbsoluteEncoder {

    int port;
    AnalogEncoder encoder;
    boolean inverted;

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
    public double getAbsolutePosition() {
        return (inverted ? -1 : 1) * (encoder.getAbsolutePosition() * 360 - 180);
    }

    @Override
    public Object getAbsoluteEncoder() {
        return encoder;
    }

}
