// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.commandGroups.AutoCommands.Routines.AutoRoutinesPID;
import frc.robot.commands.commandGroups.AutoCommands.Routines.ChargeRoutine;
import frc.robot.commands.commandGroups.AutoCommands.Routines.Autos.AutoMode;
import frc.robot.commands.commandGroups.AutoCommands.Routines.Autos.ShooterHeight;
import frc.robot.commands.teleOpCommands.Angulation.AngulationEncoder2;
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

public class RobotContainer {
  private DriveTrain driveTrain = new DriveTrain();
  private Angulation angulation = new Angulation();
  private Take take = new Take();

  public static XboxController driveController = new XboxController(OperatorConstants.kDriverControllerPort);
  public static XboxController subsystemsController = new XboxController(OperatorConstants.kSubsystemsControllerPort);

  SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    driveTrain.setDefaultCommand(new Drive(driveTrain, driveController));
    angulation.setDefaultCommand(new ManualAngle(angulation, subsystemsController));
    
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(subsystemsController, XboxController.Button.kY.value).onTrue(new ShooterHigh(take));
    new JoystickButton(subsystemsController, XboxController.Button.kX.value).onTrue(new ShooterMid(take));
    new JoystickButton(subsystemsController, XboxController.Button.kA.value).onTrue(new ShooterLow(take));
    new JoystickButton(subsystemsController, XboxController.Button.kB.value).onTrue(new AngulationEncoder2(angulation));
    new JoystickButton(subsystemsController, XboxController.Button.kRightBumper.value).whileTrue(new GetCube(take, subsystemsController));
    new JoystickButton(subsystemsController, XboxController.Button.kLeftBumper.value).whileTrue(new ThrowCube(take, subsystemsController));
  }

  public Command getAutonomousCommand() {
    //FAZER CHOOSER
    return new AutoRoutinesPID(ShooterHeight.HIGH, AutoMode.CHARGE, driveTrain, angulation, take);
  }
}
