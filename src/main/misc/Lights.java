import edu.wpi.first.wpilibj.Spark;

public class Lights {
    Spark colorSpark;

    private static enum Colors {
        
    }

    public Light(int PWMID) {
        colorSpark = new Spark(PWMID);

        colorSpark.set(-0.99);
    }
}