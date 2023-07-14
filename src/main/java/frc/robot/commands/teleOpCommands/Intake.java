// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOpCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Take;

public class Intake extends CommandBase {
  /** Creates a new InOutTake. */
  Take take = new Take();
  XboxController xboxController;
  double speed;
  int in;
  public Intake(Take take, XboxController xboxController) {
    this.take = take;
    this.xboxController = xboxController;
    addRequirements(take);
  }

  @Override
  public void initialize() {
    this.in = 1;
  }

  @Override
  public void execute() {
    double low = xboxController.getAButton() ? 1.0 : 0.0;
    double mid = xboxController.getBButton() ? 1.0 : 0.0;
    double high = xboxController.getYButton() ? 1.0 : 0.0;
    if (xboxController.getXButtonPressed()){
      in = -in;
    }
    if (low == 1){
      take.setUpperShooterVelocity(70*in);
      take.setLowerShooterVelocity(70*in);
  
    }else if(mid == 1){
      take.setUpperShooterVelocity(90*in);                                                                 
      take.setLowerShooterVelocity(120*in);

    }else if(high == 1){
      take.setUpperShooterPercentage(in);                                                                 
      take.setLowerShooterPercentage(in);
    }else{
      take.setUpperShooterPercentage(0);                                                                 
      take.setLowerShooterPercentage(0);
    }

    SmartDashboard.putNumber("Velocity Upper", take.getUpperEncoderRPM());
    SmartDashboard.putNumber("Velocity Lower", take.getLowerEncoderRPM());
    SmartDashboard.putNumber("high", high);  
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
