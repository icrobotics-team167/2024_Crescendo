package frc.robot.helpers;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysID {
    private SysIdRoutine routine;
    private DoubleSupplier velocityGetter;
    private DoubleLogEntry velocityLog;
    private DoubleSupplier positionGetter;
    private DoubleLogEntry positionLog;

    /**
     * A system identification routine class.
     * 
     * @param subsystem      The subsystem to be tested.
     * @param driveCommand   The method to drive the motor to be tested, in volts.
     * @param log            The data log.
     * @param velocityGetter The getter for the velocity of the motor.
     * @param positionGetter The getter for the position of the motor.
     */
    public SysID(Subsystem subsystem, Consumer<Measure<Voltage>> driveCommand, Consumer<SysIdRoutineLog> log,
            DoubleSupplier velocityGetter, DoubleSupplier positionGetter) {
        routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(driveCommand, log, subsystem));
        DataLog dataLog = DataLogManager.getLog();
        velocityLog = new DoubleLogEntry(dataLog, subsystem.getName() + "/DataLog/velocity");
        positionLog = new DoubleLogEntry(dataLog, subsystem.getName() + "/DataLog/position");
    }

    public void logData() {
        velocityLog.append(velocityGetter.getAsDouble());
        positionLog.append(positionGetter.getAsDouble());
    }

    public Command getIDRoutine() {
        return new ParallelCommandGroup(
                new RepeatCommand(new InstantCommand(this::logData)),
                new SequentialCommandGroup(
                        routine.quasistatic(SysIdRoutine.Direction.kForward),
                        new WaitCommand(1),
                        routine.quasistatic(SysIdRoutine.Direction.kReverse),
                        new WaitCommand(1),
                        routine.dynamic(SysIdRoutine.Direction.kForward),
                        new WaitCommand(1),
                        routine.dynamic(SysIdRoutine.Direction.kReverse)));
    }
}
