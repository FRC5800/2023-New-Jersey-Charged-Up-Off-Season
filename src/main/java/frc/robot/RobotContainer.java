// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.commandGroups.AutoCommands.Routines.AutoRoutinesPID;
import frc.robot.commands.commandGroups.AutoCommands.Routines.Autos.AutoMode;
import frc.robot.commands.teleOpCommands.Angulation.AngulationEncoder;
import frc.robot.commands.teleOpCommands.Angulation.ManualAngle;
import frc.robot.commands.teleOpCommands.Drivetrain.Drive;
import frc.robot.commands.teleOpCommands.Take.GetCube;
import frc.robot.commands.teleOpCommands.Take.ShooterHigh;
import frc.robot.commands.teleOpCommands.Take.ShooterLow;
import frc.robot.commands.teleOpCommands.Take.ShooterMid;
import frc.robot.commands.teleOpCommands.Take.ThrowCube;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

//ajustes arena de teste:
/*
 * angulo charge station
 * constantes PID (straight e turn)
 */

public class RobotContainer {
  private DriveTrain driveTrain = new DriveTrain();
  private Angulation angulation = new Angulation();
  private Take take = new Take();

  public static XboxController driveController = new XboxController(OperatorConstants.kDriverControllerPort);
  public static XboxController subsystemsController = new XboxController(OperatorConstants.kSubsystemsControllerPort);

  SendableChooser<Command> chooser = new SendableChooser<>();

  //private final Command autonomousMode = new AutoRoutinesPID(AutoMode.TESTES_CHARGESTATION, driveTrain);

  public RobotContainer() {
    driveTrain.setDefaultCommand(new Drive(driveTrain, driveController));
    angulation.setDefaultCommand(new ManualAngle(angulation, subsystemsController));
    //take.setDefaultCommand(new GetCube(take, subsystemsController));
    //chooser.setDefaultOption("Autonomous Mode", autonomousMode);
    
    configureBindings();
  }

  private void configureBindings() {
    
    new JoystickButton(subsystemsController, XboxController.Button.kY.value).onTrue(new ShooterHigh(take));
    new JoystickButton(subsystemsController, XboxController.Button.kX.value).onTrue(new ShooterMid(take));
    new JoystickButton(subsystemsController, XboxController.Button.kA.value).onTrue(new ShooterLow(take));
    new JoystickButton(subsystemsController, XboxController.Button.kB.value).onTrue(new AngulationEncoder(angulation));
    new JoystickButton(subsystemsController, XboxController.Button.kRightBumper.value).whileTrue(new GetCube(take, subsystemsController));
    new JoystickButton(subsystemsController, XboxController.Button.kLeftBumper.value).whileTrue(new ThrowCube(take, subsystemsController));
    
  }

  public Command getAutonomousCommand() {
    return new AutoRoutinesPID(AutoMode.LOW_MOB_PIECE, driveTrain, angulation, take);
    //return new FollowPathMeters(driveTrain);
    //return new ChargeRoutine(driveTrain);

    /*  var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        Constants.TrajectoryConstants.ksVolts, Constants.TrajectoryConstants.kvVoltSecondsPerMeter, Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
        DriveTrain.driveKinematics,  10);

    TrajectoryConfig config = new TrajectoryConfig(
      Constants.TrajectoryConstants.kMaxSpeedMetersPerSecond, Constants.TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveTrain.driveKinematics).addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)), 
      List.of(), 
      new Pose2d(3, 0, new Rotation2d(0)), 
      config);

     RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory, driveTrain::getPose, new RamseteController(Constants.TrajectoryConstants.kRamseteB, Constants.TrajectoryConstants.kRamseteZeta),
       new SimpleMotorFeedforward(Constants.TrajectoryConstants.ksVolts, Constants.TrajectoryConstants.kvVoltSecondsPerMeter, Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter), 
       DriveTrain.driveKinematics, driveTrain::getWheelSpeeds, new PIDController(Constants.TrajectoryConstants.kPDriveVel, 0, 0), 
       new PIDController(Constants.TrajectoryConstants.kPDriveVel, 0, 0), driveTrain::tankDriveVolts, driveTrain);
    
    driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0)); */

  }
}
