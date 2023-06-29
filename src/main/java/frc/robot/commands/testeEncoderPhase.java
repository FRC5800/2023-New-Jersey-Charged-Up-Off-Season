// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;

public class testeEncoderPhase extends CommandBase {
  DriveTrain drivetrain = new DriveTrain();
  
  double initialEncoder;
  double encoderDiff;
  double encoderGoal = Units.feetToMeters(AutoConstants.halfChargeStation + AutoConstants.halfRobotLength) + 0.400;
  boolean isBalancing = true;
  boolean inGoal = false;
  double voltage;

  public testeEncoderPhase(DriveTrain driveTrain) {
    this.drivetrain = driveTrain;
    addRequirements(drivetrain);
  }

  public double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
  
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("VoltageMotors", drivetrain.getVoltage());
    SmartDashboard.putNumber("metersCharge", encoderGoal);
    SmartDashboard.putNumber("encoder", drivetrain.getAverageEncoderMeters());
    if(isBalancing && !inGoal){
      if(drivetrain.getAverageEncoderMeters() < encoderGoal*1/3) {
        drivetrain.setVoltage(9.6);
      } else if((drivetrain.getAverageEncoderMeters() > encoderGoal*1/3) && (drivetrain.getAverageEncoderMeters() < encoderGoal)) {
        drivetrain.setVoltage(4.0);
      } else{
        inGoal = true;
      }
    }
}

  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
