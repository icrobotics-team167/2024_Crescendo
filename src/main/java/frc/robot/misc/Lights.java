package frc.robot.misc;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import frc.robot.helpers.Telemetry;
import frc.robot.helpers.Telemetry.Verbosity;

/**
 * Lights class for cool lights bruv.
*/
public class Lights {
    PWM colourSpark;

    /**
     * Cool colours.
     */
    public static enum Colours {
        RAINBOW_PALETTE(1005),
        RAINBOW_GLITTER(1055),
        CONFETTI(1065),
        SHOT_RED(1075),
        SINELON_RAINBOX_PALETTE(1105),
        SINELON_LAVA_PALETTE(1135),
        BPM_OCEAN_PALETTE(1175),
        FIRE_LARGE(1215),
        //SOLID COLORS
        HOT_PINK(1785),
        DARK_RED(1795),
        RED(1805),
        RED_ORANGE(1815),
        ORANGE(1825),
        GOLD(1835),
        YELLOW(1845),
        LAWN_GREEN(1855),
        LIME(1865),
        DARK_GREEN(1875),
        GREEN(1885),
        BLUE_GREEN(1895),
        AQUA(1905),
        SKY_BLUE(1915),
        DARK_BLUE(1925),
        BLUE(1935),
        BLUE_VIOLET(1945),
        VIOLET(1955),
        WHITE(1965),
        GRAY(1975),
        DARK_GREY(1985),
        BLACK(1995);


        int colourValue;
        /**
         * colours constructor
         * @param colourValue colour value
         */
        private Colours(int colourValue) {
            this.colourValue = colourValue;
        }
    }

    /**
     * Lights constructor
     * @param PWMID the PWMID of the blinky blinky light module (Spark motor)
     */ 
    public Lights(int PWMID) {
        colourSpark = new PWM(PWMID);
    }

    /**
     * Set the colour for the spark motor bruv
     * @param colour colours object bruv
     */
    public void setColour(Colours colour) {
        // set colour to set colourValue bruv
        colourSpark.setPulseTimeMicroseconds(colour.colourValue);
        Telemetry.sendNumber("Lights.colourValue", colourSpark.getPulseTimeMicroseconds(), Verbosity.HIGH);
    }
    public void setColorValue(int colorValue) {
        colourSpark.setPulseTimeMicroseconds(colorValue);
    }
}