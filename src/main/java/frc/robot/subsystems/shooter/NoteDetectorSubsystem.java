package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIO;
import frc.robot.subsystems.shooter.interfaceLayers.NoteDetectorIOInputsAutoLogged;

public class NoteDetectorSubsystem extends SubsystemBase {
  private final NoteDetectorIO io;
  private NoteDetectorIOInputsAutoLogged inputs;
  public NoteDetectorSubsystem(NoteDetectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/noteDetector/", inputs);
  }

  public boolean hasNote() {
    return inputs.hasNote;
  }
}
