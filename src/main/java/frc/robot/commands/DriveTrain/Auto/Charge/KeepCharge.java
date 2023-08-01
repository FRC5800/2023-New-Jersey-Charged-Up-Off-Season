// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain.Auto.Charge;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
//ajustar cálculo de speed para lados individualmente
public class KeepCharge extends CommandBase {
  private final DriveTrain driveTrain;
  private double initialSpeed = 0.4;
  private double speed = 0.4;
  /** Creates a new KeepCharge. */
  public KeepCharge(DriveTrain driveTrain) {
    this.driveTrain = driveTrain; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*speed = Math.abs(driveTrain.getRoll() * 0.6); 

    if (driveTrain.getRoll() > 4) {
      driveTrain.tankDrive(-0.5, -0.5);
    } 
    else if (driveTrain.getRoll() < -4) {
      driveTrain.tankDrive(0.5, 0.5);
    }*/

    if (driveTrain.getRoll() > 4.4) {
      driveTrain.tankDrive(-speed, -speed);
      //driveTrain.tankDrive(-0.454, -0.454);
    } 
    else if (driveTrain.getRoll() < -4.4) {
      driveTrain.tankDrive(speed, speed);
      //driveTrain.tankDrive(0.454, 0.454);
    }
    else {
      driveTrain.tankDrive(0, 0);
      speed = initialSpeed;
    }

    if (driveTrain.getRoll() > 4) {
      if (driveTrain.getAverageEncoderSpeed() < 25) {
        speed += 0.001;
      }
    } else if (driveTrain.getRoll() < -4) {
      if (driveTrain.getAverageEncoderSpeed() > -22) {
        speed += 0.001;
      }
    }

    
    SmartDashboard.putNumber("Yaw", driveTrain.getYaw());
    SmartDashboard.putNumber("Roll", driveTrain.getRoll());
    SmartDashboard.putNumber("speed", driveTrain.getAverageEncoderSpeed());

    SmartDashboard.putNumber("keepCharge", driveTrain.getRoll() );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
