package frc.robot.subsystems.shooter.interfaceLayers;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.util.SparkUtils;

public class FlywheelIOSparkFlex implements FlywheelIO {
  private final CANSparkFlex topFlywheel;
  private final CANSparkFlex bottomFlywheel;
  public FlywheelIOSparkFlex() {
    topFlywheel = new CANSparkFlex(0, MotorType.kBrushless); // TODO: Configure
    bottomFlywheel = new CANSparkFlex(0, MotorType.kBrushless);
    SparkUtils.configureSettings(false, IdleMode.kCoast, Amps.of(60), bottomFlywheel);
    SparkUtils.configureSettings(true, IdleMode.kCoast, Amps.of(60), bottomFlywheel);
  }

  @Override
  public void runSpeaker() {
    bottomFlywheel.setVoltage(12);
    topFlywheel.setVoltage(12);
  }

  @Override
  public void runAmp() {
    bottomFlywheel.setVoltage(6);
    topFlywheel.setVoltage(-6);
  }

  @Override
  public void stop() {
    topFlywheel.setVoltage(0);
    bottomFlywheel.setVoltage(0);
  }
}
