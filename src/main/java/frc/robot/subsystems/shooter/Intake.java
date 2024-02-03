package frc.robot.subsystems.shooter;

import frc.robot.abstraction.motors.AbstractMotor;
import frc.robot.helpers.MathUtils;

import com.playingwithfusion.TimeOfFlight;
//import com.playingwithfusion.TimeOfFlight.RangingMode;

import frc.robot.helpers.Telemetry;
import frc.robot.helpers.Telemetry.Verbosity;


public class Intake {
    AbstractMotor feedMotor;
    AbstractMotor intakeMotor;
    TimeOfFlight sensor;

    private static final int DISTANCE_TO_WALL = 0; //probably not 0 NEEDS TO BE IN MILLIMETERS

    public Intake(AbstractMotor motor, AbstractMotor feed, int sensorID) {
        motor.configureCurrentLimits(motor.getNominalVoltage(), motor.getPrimaryCurrentLimit(),
                motor.getSecondaryCurrentLimit());
        this.intakeMotor = motor;

        feed.configureCurrentLimits(feed.getNominalVoltage(), feed.getPrimaryCurrentLimit(),
                feed.getSecondaryCurrentLimit());
        this.feedMotor = feed;
        
        this.sensor = new TimeOfFlight(sensorID);
        this.sensor.setRangingMode(TimeOfFlight.RangingMode.Short,30);
    }

    // TODO: Figure out the design of the intake so that we can finalize methods

    public void runIntake() {
        intakeMotor.set(1);
    }

    public void stopIntake() {
    }

    public void run() {
        intakeMotor.set(1);
    }

    public void stop() {
        intakeMotor.stop();
    }

    public void runFeedIn() {
        feedMotor.set(1); // imma be real idk what set does but fuck it we ball - Tom Kalman 2024
    }

    public void runFeedOut() {
        feedMotor.set(-1); // This might be bad and broken but idk YOLO
    }

    public void stopFeed() {
        feedMotor.stop();
    }

    public boolean hasNote() {
        // TODO make not suck
        Telemetry.sendNumber("pidGet", sensor.pidGet(), Verbosity.MEDIUM);
        Telemetry.sendNumber("getRange", sensor.getRange(), Verbosity.MEDIUM); //double check these are the same, they should be but who knows.
        return (sensor.pidGet() * MathUtils.mm_TO_INCHES) < DISTANCE_TO_WALL;
    }

}
;
;
;
;
;
;
;
;
//we did it - Tom Kalman (circa 2024)