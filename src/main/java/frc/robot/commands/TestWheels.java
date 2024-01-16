package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * An auto command to test wheel actuation.
 */
public class TestWheels extends Command {
    SwerveSubsystem drivebase;
    Timer testTimer;

    /**
     * A command to test individual swerve modules.
     * 
     * @param drivebase The swerve drivebase subsystem.
     */
    public TestWheels(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;
        testTimer = new Timer();
        addRequirements(drivebase);
    }

    int currentlyTestedModule;
    int currentTestState;

    @Override
    public void initialize() {
        System.out.println("Running wheel actuation test routine.");
        currentlyTestedModule = 0;
        currentTestState = 0;
        testTimer.reset();
        testTimer.start();
        drivebase.setWheelsForward();
    }

    @Override
    public void execute() {
        if (testTimer.advanceIfElapsed(2)) {
            currentTestState++;
            System.out.println("Moving to next test. Next test: " + currentTestState);
        }
        switch (currentTestState) {
            case 0:
                drivebase.setIndividualModule(currentlyTestedModule,
                        new SwerveModuleState(0, Rotation2d.fromDegrees(90)));
                break;
            case 1:
                drivebase.setIndividualModule(currentlyTestedModule,
                        new SwerveModuleState(0, Rotation2d.fromDegrees(180)));
                break;
            case 2:
                drivebase.setIndividualModule(currentlyTestedModule,
                        new SwerveModuleState(0, Rotation2d.fromDegrees(270)));
                break;
            case 3:
                drivebase.setIndividualModule(currentlyTestedModule,
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
                break;
            case 4:
                drivebase.setIndividualModule(currentlyTestedModule,
                        new SwerveModuleState(2, Rotation2d.fromDegrees(0)));
                break;
            case 5:
                drivebase.setIndividualModule(currentlyTestedModule,
                        new SwerveModuleState(-2, Rotation2d.fromDegrees(0)));
                break;
            default:
                drivebase.setIndividualModule(currentlyTestedModule, new SwerveModuleState());
                currentlyTestedModule++;
                currentTestState = 0;
                System.out.println("Done testing module #" + currentlyTestedModule + ", moving to next module.");
                return;
        }
        drivebase.sendTelemetry();
    }

    @Override
    public boolean isFinished() {
        return currentlyTestedModule == 4;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Done testing all modules.");
        drivebase.setWheelsForward();
    }
}
