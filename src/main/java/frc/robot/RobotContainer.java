// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.BalanceCommand1;
import frc.robot.commands.Drive;
import frc.robot.commands.AutoRoutines.AutoMode;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
  DriveTrain driveTrain = new DriveTrain();
  XboxController driveController = new XboxController(0);

  SendableChooser<Command> chooser = new SendableChooser<>();

  private final Command autonomousMode = new AutoRoutines(AutoMode.ChargeStation, driveTrain);

  public RobotContainer() {
    driveTrain.setDefaultCommand(new Drive(driveTrain, driveController));

    chooser.setDefaultOption("Autonomous Mode", autonomousMode);
    SmartDashboard.putData("Auto mode", chooser);
    
    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return new BalanceCommand1(driveTrain);
  }
}
