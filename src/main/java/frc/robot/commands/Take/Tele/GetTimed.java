// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Take.Tele;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Take;

public class GetTimed extends CommandBase {
  /** Creates a new InOutTake. */
  Take take;
  double speed;
  int in;
  private double time;
  private Timer timer = new Timer();

  public GetTimed(Take take, double time) {
    this.take = take;
    this.time = time;
  
    addRequirements(take);
  }

  @Override
  public void initialize() {
    
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    this.take.setUpperShooterPercentage(0.5);
    this.take.setLowerShooterPercentage(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    take.setUpperShooterPercentage(0);                                                                 
    take.setLowerShooterPercentage(0);
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= time;
  }
}
