package frc.robot.commands.DriveTrain.Auto.trajectory;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class FollowPath extends CommandBase {
  
  private DifferentialDriveVoltageConstraint autoVoltageConstraint;
  private DriveTrain driveTrain; 
  private TrajectoryConfig config;
  private Trajectory exampleTrajectory;
  private RamseteCommand ramseteCommand;
  private String trajectoryJSON = "paths/game.wpilib.json";

  private Trajectory trajectoryWeaver;
    /** Creates a new FollowPath. */
  public FollowPath(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;

       // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        Constants.TrajectoryConstants.ksVolts, Constants.TrajectoryConstants.kvVoltSecondsPerMeter, Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
        DriveTrain.driveKinematics,  12);

    config = new TrajectoryConfig(
      Constants.TrajectoryConstants.kMaxSpeedMetersPerSecond, Constants.TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveTrain.driveKinematics).addConstraint(autoVoltageConstraint);

    /*exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)), 
      List.of(new Translation2d(1, 1)), 
      new Pose2d(2, 0, new Rotation2d(0)), 
      config); */

      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectoryWeaver = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
     } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
     }
     
    driveTrain.resetOdometry(trajectoryWeaver.getInitialPose());

     ramseteCommand = new RamseteCommand(
      trajectoryWeaver, driveTrain::getPose, new RamseteController(Constants.TrajectoryConstants.kRamseteB, Constants.TrajectoryConstants.kRamseteZeta),
       new SimpleMotorFeedforward(Constants.TrajectoryConstants.ksVolts, Constants.TrajectoryConstants.kvVoltSecondsPerMeter, Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter), 
       DriveTrain.driveKinematics, driveTrain::getWheelSpeeds, new PIDController(Constants.TrajectoryConstants.kPDriveVel, 0, 0), 
       new PIDController(Constants.TrajectoryConstants.kPDriveVel, 0, 0), driveTrain::tankDriveVolts, driveTrain);
    
    driveTrain.resetOdometry(trajectoryWeaver.getInitialPose());
    ramseteCommand.initialize();
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ramseteCommand.execute();
    //return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ramseteCommand.end(interrupted);
    driveTrain.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramseteCommand.isFinished();
//    return false;
  }
}