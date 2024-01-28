# Children of the Corn Advanced Robot Template

Template code written by [FIRST Robotics Competition Team 167, Children of the Corn](https://github.com/icrobotics-team167). This is intended to be a starting point for teams who have already gotten a fair bit of experience with writing code for FRC robots

AdvantageKit's "log literally everything" approach is used here, to allow for extremely robust debugging and simulations. In addition, it also has capabilities for high frequency odometry and vision-based pose estimation, giving it high confidence in knowing where the robot is at on the field at all times.

## License

This project uses the GNU-GPL v3 open-source license, meaning that you are free to do whatver you want with it, as long as your own project is also open-sourced under GNU-GPL v3. For example, you can take parts of this code and put it into your own project as long as that project is GNU-GPL v3. However, due to the nature of the GNU-GPL v3 license, we cannot *under any circumstances*, provide warranty for the function of this code. For more information, read the `LICENSE` file.

## Usage

- Always commit any changes before deploying code to the robot. If at an event, use the [Event Deploy For WPILib](https://marketplace.visualstudio.com/items?itemName=Mechanical-Advantage.event-deploy-wpilib) VSCode extension so that any changes are auto-committed using the "Deploy Robot Code (Event)" option. Branches should be named "event_(eventName)" such as "event_IowaRegional" so that the extension works.
- Programmers should use AdvantageScope for telemetry data, and drivers should use ShuffleBoard for match data.

## Credits

Contributors:

- [Tada Goto](https://github.com/TheComputer314)

Code taken from the following places:

- [AdvantageKit Advanced Swerve Example](https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects)
- [LimeLight Lib](https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java)
