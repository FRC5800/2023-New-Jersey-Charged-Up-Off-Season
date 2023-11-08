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

    SmartDashboard.putNumber("Contrller trigger",  xboxController.getLeftTriggerAxis());
    SmartDashboard.putNumber("Shooter speed",  speedTest);
    
    /**
     * If trying to suck ball but it's already in: speed = 0
   */
    if (!(xboxController.getPOV() == 0) && !take.getEndOfRoad()) {
      speed = 0;
      take.setUpperShooterPercentage(speed);                                                                 
      take.setLowerShooterPercentage(speed);
      return;
    }

    //get ball in 
    else if (xboxController.getPOV() == 180){
      speed = 0.55; 
      pov = true;                                                              
    }
    
    else if(xboxController.getPOV() == 0){
      speed = -0.6;
      pov = true;
    }
    else{
      speed = 0;
      pov = false;
    }

   /*if (take.getEndOfRoad() && !pov) {
      take.setUpperShooterPercentage(0);                                                                 
      take.setLowerShooterPercentage(0);
      return;
    }  */ 


    SmartDashboard.putNumber("POV", xboxController.getPOV());

    SmartDashboard.putBoolean("endofroad", take.getEndOfRoad());

    take.setLowerShooterPercentage(speed);
    take.setUpperShooterPercentage(speed);
   
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
