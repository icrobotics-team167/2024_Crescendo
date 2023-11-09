package frc.robot.helpers;

import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * A helper class for telemetry.
 */
public class Telemetry {
    public static Field2d field = new Field2d();

    /**
     * Verbosity level enum.
     */
    public static enum Verbosity {
        /**
         * No data will be sent at all. Also disables hot reload for PathPlanner.
         */
        NONE(0),
        /**
         * Only the basic data will be sent.
         */
        LOW(1),
        /**
         * Some data will be sent.
         */
        MEDIUM(2),
        /**
         * All data will be sent.
         */
        HIGH(3);

        int lvlNum;
        Verbosity(int lvlNum) {
            this.lvlNum = lvlNum;
        }
    }

    /**
     * Generic data sender for a String.
     * 
     * @param dataName       The ID in which the data will be displayed as.
     * @param data           The String data to be sent.
     * @param verbosityLevel The verbosity level of the data.
     */
    public static void sendString(String dataName, String data, Verbosity verbosityLevel) {
        if (isSendable(verbosityLevel)) {
            SmartDashboard.putString(dataName, data);
        }
    }

    /**
     * Generic data sender for a number. Will be typecasted to double.
     * 
     * @param dataName       The ID in which the data will be displayed as.
     * @param data           The double data to be sent.
     * @param verbosityLevel The verbosity level of the data.
     */
    public static void sendNumber(String dataName, double data, Verbosity verbosityLevel) {
        if (isSendable(verbosityLevel)) {
            SmartDashboard.putNumber(dataName, data);
        }
    }

    /**
     * Generic data sender for a boolean.
     * 
     * @param dataName       The ID in which the data will be displayed as.
     * @param data           The boolean data to be sent.
     * @param verbosityLevel The verbosity level of the data.
     */
    public static void sendBoolean(String dataName, boolean data, Verbosity verbosityLevel) {
        if (isSendable(verbosityLevel)) {
            SmartDashboard.putBoolean(dataName, data);
        }
    }

    /**
     * Sets the robot's pose in the field. (0, 0) is the back right corner of the
     * Blue driver station, 0 degrees rotation is facing away from the Blue driver
     * station.
     * 
     * @param robotPose The robot pose.
     */
    public static void setRobotPose(Pose2d robotPose) {
        field.setRobotPose(robotPose);
    }

    /**
     * Sets the pose of an object on the field. Creates a new field object if there
     * isn't one. (0, 0) is the back right corner of the Blue driver station, 0
     * degrees rotation is facing away from the Blue driver station.
     * 
     * @param objectName The name of the object.
     * @param objectPose The object pose.
     */
    public static void setFieldObjectPose(String objectName, Pose2d objectPose) {
        field.getObject(objectName).setPose(objectPose);
    }

    /**
     * Sends a LiveWindow object that represents the field and objects on the field,
     * such as the robot. Ignores verbosity level and always sends. Should be run
     * every robot tick for accurate data.
     */
    public static void sendField() {
        LiveWindow.enableTelemetry(field);
    }

    /**
     * Checks if a piece of telemetry should be sent or not.
     * 
     * @param verbosityLevel The telemetry data's Verbosity level enum.
     * @return If the current set Verbosity level is equal to or above the
     *         telemetry's verbosity level.
     */
    private static boolean isSendable(Verbosity verbosityLevel) {
        return verbosityLevel.lvlNum <= Constants.TELEMETRY_VERBOSITY.lvlNum;
    }
}
