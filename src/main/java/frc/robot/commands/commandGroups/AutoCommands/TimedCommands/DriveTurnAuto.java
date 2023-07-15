// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups.AutoCommands.TimedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTurnAuto extends CommandBase {
  /** Creates a new DriveTime. */
  private final DriveTrain driveTrain;
  private double angle;
  private double initialAngle;

  public DriveTurnAuto(DriveTrain driveTrain, double angle) {
    this.driveTrain = driveTrain;
    this.angle = angle;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = driveTrain.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (angle == 0) return;

    var speed = 0.8;
    var dist = (angle-Math.abs(initialAngle - driveTrain.getAngle()))/angle;
    driveTrain.tankDrive(Math.min(0.35, speed*dist), -Math.min(0.35, speed*dist));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(initialAngle - driveTrain.getAngle()) >= (angle-5) || angle == 0;
  }
}
