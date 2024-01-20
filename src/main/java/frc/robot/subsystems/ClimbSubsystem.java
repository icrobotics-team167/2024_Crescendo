package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.abstraction.motors.AbstractMotor;
import frc.robot.abstraction.motors.RevNEO500;

public class ClimbSubsystem extends SubsystemBase {
    Climb climb;

    public ClimbSubsystem(int canID){
        this.climb = new Climb(new RevNEO500(canID));
    }

    public void runForward(){
        this.climb.runForward();
    }

    public void runBackward(){
        this.climb.runBackward();
    }

    public void stop(){
        this.climb.stop();
    }
}
