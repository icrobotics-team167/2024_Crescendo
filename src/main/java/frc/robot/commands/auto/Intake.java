package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Runs the intake on the arm.
 */
public class Intake extends Command {
    private Timer timer = new Timer();
    private ArmSubsystem arm;
    private double timeout;

    /**
     * Constructs a new Intake command.
     * 
     * @param arm     The arm subsystem.
     * @param timeout How long it should run for, in seconds
     */
    public Intake(ArmSubsystem arm, double timeout) {
        this.arm = arm;
        this.timeout = timeout;
    }

    /**
     * Constructs a new Intake command.
     * 
     * <p>
     * This does not end on its own, so make sure there's a way to force this to
     * end, such as with a ParallelDeadlineGroup.
     * 
     * @param arm The arm subsystem.
     */
    public Intake(ArmSubsystem arm) {
        this(arm, -1);
    }

    @Override
    public void initialize() {
        // This method runs once at the start of the command.
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // This method runs every robot tick until the command is stopped.
        arm.intake();
    }

    @Override
    public boolean isFinished() {
        // This value is checked after execute() is ran. If true, end(false) is run.
        if (timeout <= 0) {
            return false;
        }
        return timer.hasElapsed(timeout);
    }

    @Override
    public void end(boolean interrupted) {
        // This method runs once at the end. The interrupted parameter is true when the
        // command is stopped forcefully, IE not through isFinished().
        timer.stop();
        arm.stopIntake();
    }
}
