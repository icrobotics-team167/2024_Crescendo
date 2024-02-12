package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.interfaceLayers.FeederIO;
import frc.robot.subsystems.shooter.interfaceLayers.FeederIOInputsAutoLogged;

public class FeederSubsystem extends SubsystemBase {
  private FeederIO io;
  private FeederIOInputsAutoLogged inputs;
  public FeederSubsystem(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/feeder", inputs);
  }

  public Command getFeedCommand() {
    return startEnd(io::run, io::stop);
  }
}
