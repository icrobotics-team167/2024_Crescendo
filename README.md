# Children of the Corn Advanced Robot Template

Template code written by [FIRST Robotics Competition Team 167, Children of the Corn](https://github.com/icrobotics-team167). This is intended to be a starting point for teams who have already gotten a fair bit of experience with writing code for FRC robots.

AdvantageKit's "log literally everything" approach is used here, to allow for extremely robust debugging and simulations. In addition, it also has capabilities for high frequency odometry and vision-based pose estimation, giving it high confidence in knowing where the robot is at on the field at all times.

## License

This project uses the GNU-GPL v3 open-source license. Due to the nature of the GNU-GPL v3 license, we cannot *under any circumstances*, provide warranty for the function of this code. For more information, read the `LICENSE` file or read a summary of the license [here](https://choosealicense.com/licenses/gpl-3.0/).

## General Usage

- Always commit any changes before deploying code to the robot. If at an event, use the [Event Deploy For WPILib](https://marketplace.visualstudio.com/items?itemName=Mechanical-Advantage.event-deploy-wpilib) VSCode extension so that any changes are auto-committed using the "Deploy Robot Code (Event)" option. Branches should be named "event_(eventName)" such as "event_IowaRegional" so that the extension works.
- [The Google Java Style Guide](https://google.github.io/styleguide/javaguide.html) is enforced via the [Spotless](https://github.com/diffplug/spotless) plugin for gradle. It is recommended to use the [Spotless Gradle](https://marketplace.visualstudio.com/items?itemName=richardwillis.vscode-spotless-gradle) extension for VSCode as your default formatter so that the code style guidelines are always followed.
- Programmers should use AdvantageScope for telemetry data, and drivers should use ShuffleBoard for match data.

## Credits

Contributors:

- [Tada Goto](https://github.com/TheComputer314)

Code taken from the following places:

- [AdvantageKit Advanced Swerve Example](https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects)
  - Modified to offload unit conversion math to the motor controller, improving code readability and marginally improving performance.
- [LimeLight Lib](https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java)
