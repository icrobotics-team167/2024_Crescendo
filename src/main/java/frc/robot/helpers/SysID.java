package frc.robot.helpers;

import java.util.function.Consumer;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysID {
    private SysIdRoutine routine;

    /**
     * A system identification routine class.
     * 
     * @param subsystem    The subsystem to be tested.
     * @param driveCommand The method to drive the motor to be tested, in volts.
     * @param log          The logger object.
     */
    public SysID(Subsystem subsystem, Consumer<Measure<Voltage>> driveCommand, Consumer<SysIdRoutineLog> log) {
        routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(driveCommand, log, subsystem));
    }

    public SequentialCommandGroup getIDRoutine() {
        return new SequentialCommandGroup(
                routine.quasistatic(SysIdRoutine.Direction.kForward),
                new WaitCommand(1),
                routine.quasistatic(SysIdRoutine.Direction.kReverse),
                new WaitCommand(1),
                routine.dynamic(SysIdRoutine.Direction.kForward),
                new WaitCommand(1),
                routine.dynamic(SysIdRoutine.Direction.kReverse));
    }
}
