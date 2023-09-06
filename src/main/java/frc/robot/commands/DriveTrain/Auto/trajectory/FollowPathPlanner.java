


























































































































































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
import frc.robot.commands.Take.Tele.GetTimed;
import frc.robot.commands.Take.Tele.ShooterHigh;
import frc.robot.commands.Take.Tele.ShooterLow;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

public class FollowPathPlanner extends CommandBase {
  
  private DriveTrain driveTrain; 
  private Take take;
  private HashMap<String, Command> eventMap;
  private RamseteAutoBuilder autoBuilder;
  private Command fullAuto;

  //install link: https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json 

  PathPlannerTrajectory pathPlanner = PathPlanner.loadPath("auto1", new PathConstraints(3, 2.5));
    /*Creates a new FollowPathPlanner. */
  public FollowPathPlanner(DriveTrain driveTrain, Take take, Angulation angulation) {
    this.driveTrain = driveTrain;
    this.take = take;

       // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, take);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    eventMap = new HashMap<>();
    eventMap.put("shooterHigh", new ShooterHigh(take)); 
    eventMap.put("shooterLow", new ShooterLow(take)); 
    //eventMap.put("shooter", new ShooterLow(take)); 
    eventMap.put("take", new GetTimed(take, 3));
    eventMap.put("marker",   new ShooterHigh(take));

    autoBuilder = new RamseteAutoBuilder(
      driveTrain::getPose, 
      driveTrain::resetOdometry,
      new RamseteController(Constants.TrajectoryConstants.kRamseteB, 
      Constants.TrajectoryConstants.kRamseteZeta),
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

      fullAuto = autoBuilder.fullAuto(pathPlanner);
      fullAuto.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    fullAuto.execute();
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    fullAuto.end(interrupted);
    driveTrain.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fullAuto.isFinished();
  }
}
