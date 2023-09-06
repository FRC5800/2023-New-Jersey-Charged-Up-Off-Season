package frc.robot.commands.DriveTrain.Auto.trajectory;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.Take.Tele.ShooterHigh;
import frc.robot.commands.Take.Tele.ShooterLow;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

public class FlatEventFollowPathPlanner extends CommandBase {

  //install link: https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json 

  private static PathPlannerTrajectory pathPlanner = 
    PathPlanner.loadPath("teste", new PathConstraints(3, 2.5));

    /*Creates a new EventFollowPathPlanner. */

  public static CommandBase create(DriveTrain driveTrain, Take take) {
    HashMap<String, Command> eventMap = new HashMap<>();
    //new PrintCommand("evento");
    eventMap.put("shooterLow", new ShooterLow(take)); 
    eventMap.put("shooterHigh", new ShooterHigh(take));
    eventMap.put("edu", new ShooterHigh(take));
    eventMap.put("take", new ShooterHigh(take));

    // A documentação recomenda colocar esse RamsetAutoBuilder no RobotContainer
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
      driveTrain::getPose,
      driveTrain::resetOdometry,
      new RamseteController(Constants.TrajectoryConstants.kRamseteB, Constants.TrajectoryConstants.kRamseteZeta),
      driveTrain.driveKinematics, 
      new SimpleMotorFeedforward(Constants.TrajectoryConstants.ksVolts, 
        Constants.TrajectoryConstants.kvVoltSecondsPerMeter, 
        Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
      driveTrain::getWheelSpeeds, 
      driveTrain.pidConstants,
      driveTrain::tankDriveVolts, 
      eventMap, 
      false,
      driveTrain);

      return autoBuilder.fullAuto(pathPlanner);
  }


}

