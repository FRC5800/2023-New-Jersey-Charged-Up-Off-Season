// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Take.Tele;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Take.Auto.ShooterTimedAuto;
import frc.robot.subsystems.Take;

public class GetCubeLimit extends CommandBase {
  /** Creates a new InOutTake. */
  Take take;
  XboxController xboxController;
  double speed;
  public double speedTest = 6;
  int in;
  private double time;
  private Timer timer = new Timer();


  public GetCubeLimit(Take take, double timme) {
    this.take = take;
    this.time = time;
    addRequirements(take);
  }

public double getVoltageSend(){
  return speedTest;
}
@Override
  public void initialize() {
    speed = 0.43;
    
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Contrller trigger",  xboxController.getLeftTriggerAxis());
    SmartDashboard.putNumber("Shooter speed",  speedTest);

    
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
    return take.getEndOfRoad() || timer.get() >= time;
  }
}
