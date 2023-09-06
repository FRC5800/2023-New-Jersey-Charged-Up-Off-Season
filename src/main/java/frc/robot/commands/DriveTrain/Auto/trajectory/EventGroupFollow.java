package frc.robot.commands.DriveTrain.Auto.trajectory;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.Take.Tele.ShooterHigh;
import frc.robot.commands.Take.Tele.ShooterLow;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

public class EventGroupFollow extends CommandBase {

  private DriveTrain driveTrain; 
  private Take take;
  private Angulation angulation;
  private HashMap<String, Command> eventMap;
  private RamseteAutoBuilder autoBuilder;
  private Command fullAuto;
  private FollowPathWithEvents followCommand;

  //install link: https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json 

  List<PathPlannerTrajectory> pathPlanner = PathPlanner.loadPathGroup("auto1", new PathConstraints(3, 2.5));
  //PathPlannerTrajectory pathPlanner = PathPlanner.loadPath("teste", new PathConstraints(3, 2.5));

    /*Creates a new EventGroupFOllow. */
  public EventGroupFollow(DriveTrain driveTrain, Take take, Angulation angulation) {
    this.driveTrain = driveTrain;
    this.take = take;

       // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, take);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   eventMap = new HashMap<>();
     
    //new PrintCommand("evento")
    eventMap.put("shooterLow", new ShooterLow(take)); 
    eventMap.put("shooterHigh", new ShooterHigh(take));
    eventMap.put("getCube", new ShooterLow(take));

    // A documentação recomenda colocar esse RamsetAutoBuilder no RobotContainer
    autoBuilder = new RamseteAutoBuilder(
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
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fullAuto.isFinished();
  }
}

