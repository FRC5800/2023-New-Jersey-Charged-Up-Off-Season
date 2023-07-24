// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups.AutoCommands.Charge;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ChargePigeonPID extends CommandBase {
  private final DriveTrain driveTrain;
  private double target;
  private double speed;
  private double tolerance;
  private PIDController pidController = new PIDController(0.9, 0, 0);
  

  /** Creates a new ChargeGyro. */
  public ChargePigeonPID(DriveTrain driveTrain, double target, double speed, double tolerance) {
    this.driveTrain = driveTrain;
    this.target = target;
    this.speed = speed;
    this.tolerance = tolerance;
    
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(target);
    pidController.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.tankDrive(speed, speed);
    SmartDashboard.putNumber("diff", Math.abs(driveTrain.getRoll() - target)) ;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return Math.abs(driveTrain.getRoll() - target) <= tolerance;
    return pidController.atSetpoint();
  }
}
