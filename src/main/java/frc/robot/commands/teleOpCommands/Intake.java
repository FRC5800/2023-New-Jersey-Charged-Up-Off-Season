// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOpCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Take;

public class Intake extends CommandBase {
  /** Creates a new InOutTake. */
  Take take = new Take();
  XboxController subsystemsController;
  double speed;

  public Intake(Take take, XboxController subsystemsController) {
    this.take = take;
    addRequirements(take);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    speed = subsystemsController.getRightTriggerAxis() * 0.7;
    take.setShooterSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
