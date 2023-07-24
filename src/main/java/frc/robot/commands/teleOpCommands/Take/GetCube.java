// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOpCommands.Take;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Take;

public class GetCube extends CommandBase {
  /** Creates a new InOutTake. */
  Take take;
  XboxController xboxController;
  double speed;
  int in;
  public GetCube(Take take, XboxController xboxController) {
    this.take = take;
    this.xboxController = xboxController;
    addRequirements(take);
  }

  @Override
  public void initialize() {
    speed = -0.8;
  }

  @Override
  public void execute() {
    /*if(xboxController.getAButton()){
      take.setUpperShooterPercentage(0.8);                                                                 
      take.setLowerShooterPercentage(0.8);
    } else {
      take.setUpperShooterPercentage(0);                                                                 
      take.setLowerShooterPercentage(0);
    }*/

    take.setUpperShooterPercentage(speed);                                                                 
    take.setLowerShooterPercentage(speed);
  }



  @Override
  public void end(boolean interrupted) {
    take.setUpperShooterPercentage(0);                                                                 
    take.setLowerShooterPercentage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
