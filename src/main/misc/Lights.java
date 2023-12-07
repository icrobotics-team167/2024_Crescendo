import edu.wpi.first.wpilibj.Spark;

/**
 * Lights class for cool lights bruv.
*/
public class Lights {
    Spark colourSpark;

    /**
     * Cool colours bruv.
     */
    private static enum Colours {
        // cool colours. NOT COLORS BRUV!!!!!!!
        RAINBOWPALETTE(-0.99),
        RAINBOWGLITTER(-0.89),
        CONFETTI(-0.87),
        SHOTRED(-0.85),
        SINELONRAINBOXPALETTE(-0.79),
        SINELONLAVAPALETTE(-0.73),
        BPMOCEANPALETTE(-0.75),
        FIRELARGE(-0.57),
        TWINKLESPARTYPALETTE(-0.53),
        LARSONSCANNERGRAY(-0.33),
        LIGHTCHASEBLUE(-0.29),
        HEARTBEATBLUE(-0.23),
        BREATHGRAY(-0.13),
        STROBEGOLD(-0.07),
        HOTPINKSOLID(0.57),
        DARKREDSOLID(0.59),
        REDSOLID(0.61),
        GOLDSOLID(0.67),
        LAWNGREENSOLID(0.71),
        LIMESOLID(0.73),
        DARKGREENSOLID(0.75);

        double colourValue;
        /**
         * Colours constructor bruv
         * @param colourValue colour value bruv
         */
        private Colours(double colourValue) {
            this.colourValue = colourValue;
        }
    }

    /**
     * Loit contstructor bruv
     * @param PWMID the PWMID of the blinky blinky light module bruv (Spark motor)
     */ 
    public Light(int PWMID) {
        colourSpark = new Spark(PWMID);
    }

    /**
     * Set the colour for the spark motor bruv
     * @param colour Colours object bruv
     */
    public void setColour(Colours colour) {
        // set colour to set colourValue bruv
        colourSpark.set(colour.colourValue);
    }
}