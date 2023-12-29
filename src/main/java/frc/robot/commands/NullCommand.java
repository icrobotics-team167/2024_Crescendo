package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A blank command that does nothing and never ends. Intended to be a template.
 */
public class NullCommand extends Command {
    public NullCommand() {
        // addRequirements(Fill this out with subsystems that this command uses);
        setName("Null Command");
    }

    @Override
    public void initialize() {
        // This method runs once at the start of the command.
    }

    @Override
    public void execute() {
        // This method runs every robot tick until isFinished() is true.
    }

    @Override
    public boolean isFinished() {
        // This value is checked after execute() is ran. If true, end(false) is run.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // This method runs once at the end. The interrupted parameter is true when the command is stopped forcefully, IE not through isFinished().
    }
}
