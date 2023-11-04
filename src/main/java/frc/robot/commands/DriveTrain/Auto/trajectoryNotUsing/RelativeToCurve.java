// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain.Auto.trajectoryNotUsing;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class RelativeToCurve extends CommandBase {
  /** Creates a new GoToBottomMiddle. */
  
  private DifferentialDriveVoltageConstraint autoVoltageConstraint;
  private DriveTrain driveTrain; 
  private TrajectoryConfig config;
  private Trajectory trajectoryBase;
  private Trajectory trajectory;
  private Pose2d initialPose;
  private RamseteCommand ramseteCommand;

  public 
  RelativeToCurve(DriveTrain driveTrain, Pose2d initialPose) {
    this.driveTrain = driveTrain;
    this.initialPose = initialPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        
    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        Constants.TrajectoryConstants.ksVolts, 
        Constants.TrajectoryConstants.kvVoltSecondsPerMeter, 
        Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
        driveTrain.driveKinematics,  12);
  
    config = new TrajectoryConfig(
      Constants.TrajectoryConstants.kMaxSpeedMetersPerSecond, 
      Constants.TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(driveTrain.driveKinematics).addConstraint(autoVoltageConstraint);

    trajectoryBase = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d()), 
      List.of(new Translation2d(1, 0), new Translation2d(1, 1), new Translation2d(0, 1)),
      new Pose2d(0, 2, new Rotation2d(0)), 
      config); 

      Pose2d bOrigin = initialPose;
      trajectory = trajectoryBase.relativeTo(bOrigin);

      
    ramseteCommand = new RamseteCommand(
      trajectory, driveTrain::getPose, 
      new RamseteController(Constants.TrajectoryConstants.kRamseteB, Constants.TrajectoryConstants.kRamseteZeta),
       new SimpleMotorFeedforward(Constants.TrajectoryConstants.ksVolts, 
        Constants.TrajectoryConstants.kvVoltSecondsPerMeter, 
        Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter), 
       driveTrain.driveKinematics, 
       driveTrain::getWheelSpeeds, 
       new PIDController(Constants.TrajectoryConstants.kPDriveVel, 0, 0), 
       new PIDController(Constants.TrajectoryConstants.kPDriveVel, 0, 0), 
       driveTrain::tankDriveVolts, driveTrain);
   
   ramseteCommand.initialize();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ramseteCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ramseteCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return ramseteCommand.isFinished();
  }
}
