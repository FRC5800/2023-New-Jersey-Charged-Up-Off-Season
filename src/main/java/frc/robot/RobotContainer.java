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
import frc.robot.commands.commandGroups.AutoRoutinesPID;
import frc.robot.commands.commandGroups.AutoRoutinesTimed;
import frc.robot.commands.commandGroups.Autos.AutoModeTimed;
import frc.robot.commands.commandGroups.Autos.AutoModePID;
import frc.robot.commands.teleOpCommands.Angle;
import frc.robot.commands.teleOpCommands.AngulationEncoder;
import frc.robot.commands.teleOpCommands.Drive;
import frc.robot.commands.teleOpCommands.GetCube;
import frc.robot.commands.teleOpCommands.ShooterMid;
import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

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
    take.setDefaultCommand(new GetCube(take, subsystemsController));
    new JoystickButton(subsystemsController, XboxController.Button.kB.value).onTrue(new ShooterMid(take));

    //chooser.setDefaultOption("Autonomous Mode", autonomousMode);
    SmartDashboard.putData("Auto mode", chooser);
    
    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return new AngulationEncoder(angulation);
  }
}
