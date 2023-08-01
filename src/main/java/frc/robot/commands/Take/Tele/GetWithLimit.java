// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Take.Tele;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

public class GetWithLimit extends CommandBase {
  
  private Take take;
  private final double SPEED = 0.3;

  public GetWithLimit(Take take) {
    this.take = take;
    addRequirements(take);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.take.setUpperShooterPercentage(SPEED);
    this.take.setLowerShooterPercentage(SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.take.setUpperShooterPercentage(0);
    this.take.setLowerShooterPercentage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.take.getEndOfRoad();
  }
}
