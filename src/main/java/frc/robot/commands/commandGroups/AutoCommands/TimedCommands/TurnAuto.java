// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups.AutoCommands.TimedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class TurnAuto extends CommandBase {
  /** Creates a new TurnAuto. */

  private final DriveTrain driveTrain;
  private double ang;
  double sensorAngle;
  double outputSpeed;
  private PIDController pidController = new PIDController(Constants.DriveConstants.KPTurnAuto, Constants.DriveConstants.KITurnAuto, Constants.DriveConstants.KDTurnAuto);
  public TurnAuto(DriveTrain driveTrain, double ang) {
    this.driveTrain = driveTrain;
    this.ang = ang;

    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetGyro();
    pidController.reset();
    pidController.enableContinuousInput(-DriveConstants.ContinuousInputTurnAuto, DriveConstants.ContinuousInputTurnAuto);
    pidController.setIntegratorRange(-DriveConstants.IrangeTurnAuto, DriveConstants.IrangeTurnAuto); 
    pidController.setSetpoint(ang);
    pidController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sensorAngle = driveTrain.getAngle();
    
    outputSpeed = MathUtil.clamp(pidController.calculate(sensorAngle, ang), -0.8, 0.8);
    driveTrain.tankDrive(outputSpeed, -outputSpeed);

    SmartDashboard.putNumber("Gyro Value", sensorAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
