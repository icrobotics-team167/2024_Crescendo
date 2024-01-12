import edu.wpi.first.wpilibj.Spark;

/**
 * Lights class for cool lights bruv.
*/
public class Lights {
    Spark colourSpark;

    /**
     * Cool colours.
     */
    private static enum Colours {
        RAINBOW_PALETTE(-0.99),
        RAINBOW_GLITTER(-0.89),
        CONFETTI(-0.87),
        SHOT_RED(-0.85),
        SINELON_RAINBOX_PALETTE(-0.79),
        SINELON_LAVA_PALETTE(-0.73),
        BPM_OCEAN_PALETTE(-0.75),
        FIRE_LARGE(-0.57),
        TWINKLES_PARTY_PALETTE(-0.53),
        LARSON_SCANNER_GRAY(-0.33),
        LIGHT_CHASEB_LUE(-0.29),
        HEARTBEAT_BLUE(-0.23),
        BREATH_GRAY(-0.13),
        STROBE_GOLD(-0.07),
        HOT_PINK_SOLID(0.57),
        DARK_RED_SOLID(0.59),
        RED_SOLID(0.61),
        GOLD_SOLID(0.67),
        LAWN_GREEN_SOLID(0.71),
        LIME_SOLID(0.73),
        DARK_GREEN_SOLID(0.75);

        double colourValue;
        /**
         * colours constructor
         * @param colourValue colour value
         */
        private colours(double colourValue) {
            this.colourValue = colourValue;
        }
    }

    /**
     * Lights constructor
     * @param PWMID the PWMID of the blinky blinky light module (Spark motor)
     */ 
    public Light(int PWMID) {
        colourSpark = new Spark(PWMID);
    }

    /**
     * Set the colour for the spark motor bruv
     * @param colour colours object bruv
     */
    public void setColour(colours colour) {
        // set colour to set colourValue bruv
        colourSpark.set(colour.colourValue);
    }
}