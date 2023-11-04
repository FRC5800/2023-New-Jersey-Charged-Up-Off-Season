// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Take.Tele;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Take.Auto.ShooterTimedAuto;
import frc.robot.subsystems.Take;

public class GetCube extends CommandBase {
  /** Creates a new InOutTake. */
  Take take;
  XboxController xboxController;
  double speed;
  public double speedTest = 6;
  int in;
  private boolean pov;

  
  public GetCube(Take take, XboxController xboxController) {
    this.take = take;
    this.xboxController = xboxController;
    pov = false;
    addRequirements(take);
  }

  

public double getVoltageSend(){
  return speedTest;
}
@Override
  public void initialize() {
    speed = 0;
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
    SmartDashboard.putNumber("Contrller trigger",  xboxController.getLeftTriggerAxis());
    SmartDashboard.putNumber("Shooter speed",  speedTest);
    
    if (xboxController.getPOV() == 0){
      take.setUpperShooterPercentage(0.65);   
      take.setLowerShooterPercentage(0.65);   
      pov = true;                                                              
    }else if(xboxController.getPOV() == 180){
      take.setLowerShooterPercentage(-0.65);
      take.setUpperShooterPercentage(-0.65);
      pov = true;
    }else{
      take.setUpperShooterPercentage(0);                                                                 
      take.setLowerShooterPercentage(0);
      pov = false;
    }

   /*if (take.getEndOfRoad() && !pov) {
      take.setUpperShooterPercentage(0);                                                                 
      take.setLowerShooterPercentage(0);
      return;
    }  */ 

    if (xboxController.getRightBumper()) {
      speed = 0.35;
    } else {
      if (xboxController.getLeftTriggerAxis() > 0.15) {
        speed = 0.4 * xboxController.getLeftTriggerAxis();
      }else{
        speed = 0;
      }
    }

    SmartDashboard.putNumber("POV", xboxController.getPOV());

   
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
