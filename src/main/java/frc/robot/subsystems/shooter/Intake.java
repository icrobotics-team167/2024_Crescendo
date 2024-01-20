package frc.robot.subsystems.shooter;

import frc.robot.abstraction.motors.AbstractMotor;

public class Intake {
    AbstractMotor feedMotor;
    AbstractMotor intakeMotor;

    public Intake(AbstractMotor motor, AbstractMotor feed) {
        motor.configureCurrentLimits(motor.getNominalVoltage(), motor.getPrimaryCurrentLimit(),
                motor.getSecondaryCurrentLimit());
        this.intakeMotor = motor;

        feed.configureCurrentLimits(feed.getNominalVoltage(), feed.getPrimaryCurrentLimit(), feed.getSecondaryCurrentLimit());
        this.feedMotor = feed;
    }

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
        feedMotor.set(-1); //This might be bad and broken but idk YOLO
    }

    public void stopFeed() {
        feedMotor.stop();
    }

}
