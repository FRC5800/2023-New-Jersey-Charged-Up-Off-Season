// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trash;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveAuto extends CommandBase {
  DriveTrain driveTrain = new DriveTrain();
  PIDController pidController = new PIDController(Constants.AutoConstants.KP_DRIVEAUTO,Constants.AutoConstants.KI_DRIVEAUTO,0);
  double setPoint;
  double encoderMeters;
  /** Creates a new DriveAuto. */
  public DriveAuto(DriveTrain driveTrain, double setPoint) {
    this.driveTrain = driveTrain;
    this.setPoint = setPoint;
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    pidController.setSetpoint(setPoint);
    pidController.setTolerance(0.05);
    encoderMeters = driveTrain.getAverageEncoderMeters();
  }

  @Override
  public void execute() {
    double speed = pidController.calculate(encoderMeters);
    driveTrain.tankDrive(speed, speed);
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
