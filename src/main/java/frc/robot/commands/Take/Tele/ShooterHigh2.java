// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Take.Tele;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Take;

public class ShooterHigh2 extends CommandBase {
  /** Creates a new ShooterHigh2. */
  Take take;
  double speed;
  
  private double time;
  private Timer timer = new Timer();


  public ShooterHigh2(Take take) {
    this.take = take;
    this.time = 1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(take);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = -0.8;
    take.setLowerShooterPercentage(0);
    take.setUpperShooterPercentage(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    take.setLowerShooterPercentage(-0.9);
    take.setUpperShooterPercentage(-0.7);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    take.setLowerShooterPercentage(0);
    take.setUpperShooterPercentage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= time;
  }
}
