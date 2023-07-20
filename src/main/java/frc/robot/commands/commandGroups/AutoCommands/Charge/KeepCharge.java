// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups.AutoCommands.Charge;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class KeepCharge extends CommandBase {
  private final DriveTrain driveTrain;
  private double initialAngle;
  private double speed;
  /** Creates a new KeepCharge. */
  public KeepCharge(DriveTrain driveTrain) {
    this.driveTrain = driveTrain; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*speed = Math.abs(driveTrain.getRoll() * 0.6); 

    if (driveTrain.getRoll() > 4) {
      driveTrain.tankDrive(-0.5, -0.5);
    } 
    else if (driveTrain.getRoll() < -4) {
      driveTrain.tankDrive(0.5, 0.5);
    }*/

    if (driveTrain.getRoll() > 5) {
      driveTrain.setVoltage(-2.7);
      //driveTrain.tankDrive(-0.454, -0.454);
    } 
    else if (driveTrain.getRoll() < -5) {
      driveTrain.setVoltage(2.7);
      //driveTrain.tankDrive(0.454, 0.454);
    }
    else {
      driveTrain.tankDrive(0, 0);
    }

    SmartDashboard.putNumber("keepCharge", driveTrain.getRoll() );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
