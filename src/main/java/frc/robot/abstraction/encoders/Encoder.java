package frc.robot.abstraction.encoders;

/**
 * An abstraction class for encoders.
 */
public class Encoder {
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

}
