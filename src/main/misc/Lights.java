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
        RAINBOWPALETTE,
        RAINBOWGLITTER,
        CONFETTI,
        SHOTRED,
        SINELONRAINBOXPALETTE,
        SINELONLAVAPALETTE,
        BPMOCEANPALETTE,
        FIRELARGE,
        TWINKLESPARTYPALETTE,
        LARSONSCANNERGRAY,
        LIGHTCHASEBLUE,
        HEARTBEATBLUE,
        BREATHGRAY,
        STROBEGOLD,
        HOTPINKSOLID,
        DARKREDSOLID,
        REDSOLID,
        GOLDSOLID,
        LAWNGREENSOLID,
        LIMESOLID,
        DARKGREENSOLID;

        double colourValue;
        /**
         * Colours constructor 
         * @param colourValue colour value bruv
         */
        private Colours(double colourValue) {
            this.colorValue = colourValue;
        }
    }

    /**
     * Loit contstructor bruv
     * @param PWMID the PWMID of the blinky blinky light module bruv (Spark motor)
     */ 
    public Light(int PWMID) {
        colourSpark = new Spark(PWMID);

        colourSpark.set(-0.99);
    }
}