package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AimManualOverride extends Command {
    DoubleSupplier pivotSupplier;
    ShooterSubsystem shooter;
    public AimManualOverride(DoubleSupplier pivotSupplier, ShooterSubsystem shooter) {
        this.pivotSupplier = pivotSupplier;
        this.shooter = shooter;

        addRequirements(shooter);
        setName("Aiming Manual Override");
    }

    @Override
    public void execute() {
        shooter.runPivot(pivotSupplier.getAsDouble());
    }
}
