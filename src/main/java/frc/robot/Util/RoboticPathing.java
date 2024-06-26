package frc.robot.Util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;

public class RoboticPathing {
  // Load the path we want to pathfind to and follow
/*   public PathPlannerPath topPath = PathPlannerPath.fromPathFile("Top to Amp");
  public PathPlannerPath midPath = PathPlannerPath.fromPathFile("Mid to Amp");
  public PathPlannerPath botPath = PathPlannerPath.fromPathFile("Bot to Amp");
  public PathPlannerPath topSource = PathPlannerPath.fromPathFile("Top Source");
  public PathPlannerPath midSource = PathPlannerPath.fromPathFile("Mid Source");
  public PathPlannerPath botSource = PathPlannerPath.fromPathFile("Bot Source");
  public PathPlannerPath topSpeaker = PathPlannerPath.fromPathFile("Top Speaker");
  public PathPlannerPath midSpeaker = PathPlannerPath.fromPathFile("Mid Speaker");
  public PathPlannerPath botSpeaker = PathPlannerPath.fromPathFile("Bot Speaker"); */

  public Pose2d blueAmp = new Pose2d(1.85, 7.72, new Rotation2d(Units.degreesToRadians(-90)));
  public Pose2d blueSubwoofer = new Pose2d(1.27, 5.51, new Rotation2d(Units.degreesToRadians(0)));

  // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
  public PathConstraints constraints = new PathConstraints(
          1.0, 1.0,
          Math.PI, Units.degreesToRadians(0));

/*   // Since AutoBuilder is configured, we can use it to build pathfinding commands
  public Command TopAmpRobotic = AutoBuilder.pathfindThenFollowPath(
          topPath,
          constraints,
          1.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command MidAmpRobotic = AutoBuilder.pathfindThenFollowPath(
          midPath,
          constraints,
          1.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command BotAmpRobotic = AutoBuilder.pathfindThenFollowPath(
          botPath,
          constraints,
          1.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command topRobotic = AutoBuilder.pathfindThenFollowPath(
          topSpeaker,
          constraints,
          0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command midRobotic = AutoBuilder.pathfindThenFollowPath(
          midSpeaker,
          constraints,
          0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command botRobotic = AutoBuilder.pathfindThenFollowPath(
          botSpeaker,
          constraints,
          0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command topSourceRobotic = AutoBuilder.pathfindThenFollowPath(
          topSource,
          constraints,
          0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command midSourceRobotic = AutoBuilder.pathfindThenFollowPath(
          midSource,
          constraints,
          0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command botSourceRobotic = AutoBuilder.pathfindThenFollowPath(
          botSource,
          constraints,
          0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  ); */

  public Command pathToAmp = AutoBuilder.pathfindToPoseFlipped(blueAmp, constraints);

  public Command pathToSub = AutoBuilder.pathfindToPoseFlipped(blueSubwoofer, constraints);
}
