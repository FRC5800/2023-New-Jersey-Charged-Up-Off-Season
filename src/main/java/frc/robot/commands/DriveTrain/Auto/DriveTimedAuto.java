// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain.Auto;


import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTimedAuto extends CommandBase {
  /** Creates a new DriveTime. */
  private final DriveTrain driveTrain;
  private double time;
  private double leftMotorPercentage;
  private double rightMotorPercentage;
  private Timer timer = new Timer();

  public final static DriveTimedAuto MOB(DriveTrain driveTrain) {
    return new DriveTimedAuto(driveTrain, 2.8, -0.756, -0.75);
  }

  public final static DriveTimedAuto CHARGE(DriveTrain driveTrain) {
    return new DriveTimedAuto(driveTrain, 2.25, -0.75, -0.75);
  }
  
  public final static DriveTimedAuto MOB_CHARGE(DriveTrain driveTrain) {
    return new DriveTimedAuto(driveTrain, 3.4, 0.6, 0.6);
  }

  public final static DriveTimedAuto GRID(DriveTrain driveTrain) {
    return new DriveTimedAuto(driveTrain, 4, 0.7, 0.7);
  }

  public DriveTimedAuto(DriveTrain driveTrain, double time, double leftMotorPercentage, double rightMotorPercentage) {
    this.driveTrain = driveTrain;
    this.leftMotorPercentage = leftMotorPercentage;
    this.rightMotorPercentage = rightMotorPercentage;
    this.time = time;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.tankDrive(leftMotorPercentage, rightMotorPercentage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
