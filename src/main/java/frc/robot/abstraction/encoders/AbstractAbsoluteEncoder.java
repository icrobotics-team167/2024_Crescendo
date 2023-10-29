package frc.robot.abstraction.encoders;

/**
 * An abstraction class for encoders.
 */
public abstract class AbstractAbsoluteEncoder {
    /**
     * The maximum number of times that the encoder will attempt to apply
     * configurations,
     * if it takes more than this amount of tries assume there's something wrong and
     * abort.
     */
    public final int maximumRetries = 5;

    /**
     * If the last angle reading was fault or not. Should always be false normally.
     */
    public final boolean readingError = false;

    /**
     * Reset the encoder to factory defaults.
     */
    public abstract void factoryDefault();

    /**
     * Clear sticky faults on the encoder.
     */
    public abstract void clearStickyFaults();

    /**
     * Configure the absolute encoder to read from [-180, 180) per second.
     *
     * @param inverted Whether the encoder is inverted.
     */
    public abstract void configure(boolean inverted);

    /**
     * Get the absolute position of the encoder.
     *
     * @return Absolute position in degrees from [-180, 180).
     */
    public abstract double getAbsolutePosition();

    /**
     * Get the instantiated absolute encoder Object.
     *
     * @return Absolute encoder object.
     */
    public abstract Object getAbsoluteEncoder();
}
