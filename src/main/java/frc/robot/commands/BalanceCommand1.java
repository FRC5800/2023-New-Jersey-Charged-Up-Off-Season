// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class BalanceCommand1 extends CommandBase {
  DriveTrain drivetrain = new DriveTrain();
  
  double initialPitch;
  double pitch;
  boolean isBalancing = false;
  boolean inEncoderMode = false;

  public BalanceCommand1(DriveTrain driveTrain) {
    this.drivetrain = driveTrain;
    
    addRequirements(drivetrain);
  }

  public double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
  
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    initialPitch = drivetrain.ahrs.getPitch();
  }

  @Override
  public void execute() {
    pitch = drivetrain.ahrs.getPitch();
    if(isBalancing == false && pitch > initialPitch + 1) {
      isBalancing = true;
    }

    if(!isBalancing) {
      drivetrain.tankDrive(0.5, 0.5);
      drivetrain.resetEncoders();
    }
    if(isBalancing){
      if(drivetrain.getAverageEncoderMeters() < 1350) {
        drivetrain.tankDrive(0.5, 0.5);
      } else {
        drivetrain.tankDrive(clamp(drivetrain.ahrs.getPitch()/8, -0.4, 0.4), clamp(drivetrain.ahrs.getPitch()/8, -0.4, 0.4));
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
