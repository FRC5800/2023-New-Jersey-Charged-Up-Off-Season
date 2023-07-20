// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    //return new AutoRoutinesPID(AutoMode.MID_MOB, driveTrain, angulation, take);
    //return new ChargeRoutine(driveTrain);
    return DrivePIDAuto.MOB(driveTrain);

  }
}
