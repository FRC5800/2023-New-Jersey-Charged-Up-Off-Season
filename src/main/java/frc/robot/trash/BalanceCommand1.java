// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trash;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceConstants;
import frc.robot.subsystems.DriveTrain;

public class BalanceCommand1 extends CommandBase {
  DriveTrain drivetrain = new DriveTrain();
  
  double initialPitch;
  double initialEncoder;
  double encoderDiff;
  double pitch;
  double encoderGoal = Units.feetToMeters(BalanceConstants.halfChargeStation + BalanceConstants.halfRobotLength) +400;
  boolean isBalancing = true;
  boolean inGoal = false;
  double voltage;

  public BalanceCommand1(DriveTrain driveTrain) {
    this.drivetrain = driveTrain;
    addRequirements(drivetrain);
  }
  
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    initialPitch = drivetrain.getPitch();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("encoder", drivetrain.getAverageEncoderMeters());
    SmartDashboard.putNumber("Pigeon Pitch", drivetrain.getPitch());
    pitch = drivetrain.getPitch();
    encoderDiff = initialEncoder - drivetrain.getAverageEncoderMeters();

    if(!isBalancing && pitch > initialPitch + 4) {
      isBalancing = true;
      initialEncoder = drivetrain.getAverageEncoderMeters();
    } else {
      drivetrain.tankDrive(0.7, 0.7);
    }

    if(isBalancing && !inGoal){
      if(drivetrain.getAverageEncoderMeters() < encoderGoal*1/3) {
        drivetrain.setVoltage(9.6);
      } else if((drivetrain.getAverageEncoderMeters() > encoderGoal*1/3) && (drivetrain.getAverageEncoderMeters() < encoderGoal)) {
        drivetrain.setVoltage(4.0);
      } else{
        inGoal = true;
      }
    }else if(pitch < initialPitch - 1) {
      drivetrain.setVoltage(4.0);
    } else if(pitch > initialPitch + 1) {
      drivetrain.setVoltage(-4.0);
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
