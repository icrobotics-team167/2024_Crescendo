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
        FIRE_LARGE(1215);

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
        colourSpark = new PWM(0);
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
}