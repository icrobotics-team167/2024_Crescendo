package frc.robot.abstraction.encoders;

import edu.wpi.first.math.geometry.Rotation2d;

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
     * Configure whether the encoder is inverted or not.
     *
     * @param inverted Whether the encoder is inverted.
     */
    public abstract void configureInverted(boolean inverted);

    /**
     * Configure the encoder's offset.
     * 
     * @param offset The encoder offset, as a Rotation 2d object.
     */
    public abstract void configureOffset(Rotation2d offset);

    /**
     * Get the absolute position of the encoder.
     *
     * @return Absolute position as a Rotation2d object.
     */
    public abstract Rotation2d getAbsolutePosition();

    /**
     * Get the instantiated absolute encoder Object.
     *
     * @return Absolute encoder object.
     */
    public abstract Object getAbsoluteEncoder();
}
