// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.commandGroups.AutoRoutinesPID;
import frc.robot.commands.commandGroups.AutoRoutinesTimed;
import frc.robot.commands.commandGroups.ChargeRoutine;
import frc.robot.commands.commandGroups.AutoCommands.PIDCommands.DrivePIDAuto;
import frc.robot.commands.commandGroups.Autos.AutoMode;
import frc.robot.commands.teleOpCommands.Angle;
import frc.robot.commands.teleOpCommands.AngulationEncoder;
import frc.robot.commands.teleOpCommands.AngulationEncoderBom;
import frc.robot.commands.teleOpCommands.Drive;
import frc.robot.commands.teleOpCommands.GetCube;
import frc.robot.commands.teleOpCommands.ShooterHigh;
import frc.robot.commands.teleOpCommands.ShooterLow;
import frc.robot.commands.teleOpCommands.ShooterMid;
import frc.robot.commands.teleOpCommands.ThrowCube;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autoCommands.testeEncoderPhase;

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
    angulation.setDefaultCommand(new Angle(angulation, subsystemsController));
    //take.setDefaultCommand(new GetCube(take, subsystemsController));
    new JoystickButton(subsystemsController, XboxController.Button.kB.value).onTrue(new ShooterMid(take));
    

    subsystemsController.getXButton();
    //chooser.setDefaultOption("Autonomous Mode", autonomousMode);
    SmartDashboard.putData("Auto mode", chooser);
    SmartDashboard.putNumber("pitch", driveTrain.getPitch());
    SmartDashboard.putNumber("angulation position", angulation.getEncoderRotations());

    
    configureBindings();
  }

  private void configureBindings() {
    //OperatorConstants.buttonX.toggleOnTrue(new AngulationEncoderBom(angulation));
    new JoystickButton(subsystemsController, XboxController.Button.kY.value).onTrue(new ShooterHigh(take));
    new JoystickButton(subsystemsController, XboxController.Button.kX.value).onTrue(new ShooterMid(take));
    new JoystickButton(subsystemsController, XboxController.Button.kA.value).onTrue(new ShooterLow(take));
    new JoystickButton(subsystemsController, XboxController.Button.kB.value).onTrue(new AngulationEncoderBom(angulation));

    new JoystickButton(subsystemsController, XboxController.Button.kRightBumper.value).whileTrue(new GetCube(take, subsystemsController));
    new JoystickButton(subsystemsController, XboxController.Button.kLeftBumper.value).whileTrue(new ThrowCube(take, subsystemsController));  //onTrue(new ThrowCube(take, subsystemsController));
    
  }

  public Command getAutonomousCommand() {
    return new AutoRoutinesPID(AutoMode.PATH, driveTrain, angulation, take);
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
