package frc.robot.commands.DriveTrain.Auto.trajectory;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Take.Tele.ShooterHigh;
import frc.robot.commands.Take.Tele.ShooterLow;
import frc.robot.commands.Take.Tele.ShooterMid;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

public class EventFollowPathPlanner extends CommandBase {

  private DriveTrain driveTrain; 
  private Take take;
  private Angulation angulation;
  private HashMap<String, Command> eventMap;
  private RamseteAutoBuilder autoBuilder;
  private Command fullAuto;
  private FollowPathWithEvents followCommand;

  //install link: https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json 

  PathPlannerTrajectory pathPlanner = 
    PathPlanner.loadPath("teste", new PathConstraints(3, 2.5));

    /*Creates a new EventFollowPathPlanner. */
  public EventFollowPathPlanner(DriveTrain driveTrain, Take take) {
    this.driveTrain = driveTrain;
    this.take = take;

       // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    eventMap = new HashMap<>();
    //new PrintCommand("evento");
    eventMap.put("evento", new ShooterHigh(take)); 
    eventMap.put("shooterStart", new ShooterLow(take)); 
    eventMap.put("shooter", new ShooterHigh(take));
    eventMap.put("event", new ShooterHigh(take));

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
      driveTrain, take);

      fullAuto = autoBuilder.fullAuto(pathPlanner);
      followCommand = new FollowPathWithEvents(
        fullAuto, pathPlanner.getMarkers(), eventMap);
      followCommand.initialize();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    followCommand.execute();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    followCommand.end(interrupted);
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return followCommand.isFinished();
  }
}

