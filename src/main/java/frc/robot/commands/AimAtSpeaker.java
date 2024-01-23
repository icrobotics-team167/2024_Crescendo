package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Vision.LimeLight;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;

public class AimAtSpeaker extends Command {
    ShooterSubsystem shooter;
    DoubleConsumer rotationalOverrideConsumer;
    Runnable rotationalOverrideDisabler;

    PIDController rotationalOverridePID = new PIDController(1.0 / 75.0, 0, 0);
    PIDController pivotPID = new PIDController(0.9, 0, 0);

    public AimAtSpeaker(ShooterSubsystem shooter, DoubleConsumer rotationalOverrideConsumer,
            Runnable rotationalOverrideDisabler) {
        this.shooter = shooter;
        this.rotationalOverrideConsumer = rotationalOverrideConsumer;
        this.rotationalOverrideDisabler = rotationalOverrideDisabler;

        addRequirements(shooter);
        setName("Aim At Speaker");
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(LimeLight.APRILTAG_DETECTOR)) {
            return;
        }
        double rotError = LimelightHelpers.getTX(LimeLight.APRILTAG_DETECTOR);
        double pivotError = LimelightHelpers.getTY(LimeLight.APRILTAG_DETECTOR);

        rotationalOverrideConsumer.accept(rotationalOverridePID.calculate(rotError, 0));
        shooter.runPivot(pivotPID.calculate(pivotError, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        rotationalOverrideDisabler.run();
    }
}
