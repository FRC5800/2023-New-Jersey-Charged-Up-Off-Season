// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain.Auto;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.AutoConstants;

public class TurnAutoPID extends CommandBase {
  private final DriveTrain driveTrain;
  private double ang;
  private double sensorAngle;
  private double outputSpeed;
  private PIDController pidController = 
  new PIDController(Constants.AutoConstants.KP_TURNAUTO, Constants.AutoConstants.KI_TURNAUTO, Constants.AutoConstants.KD_TURNAUTO);

  public TurnAutoPID(DriveTrain driveTrain, double ang) {
    this.driveTrain = driveTrain;
    this.ang = -ang;

    addRequirements(driveTrain);
  }
  

  @Override
  public void initialize(){
    driveTrain.resetGyro();
    pidController.reset();
    pidController.enableContinuousInput(-AutoConstants.CONTINIUS_INPUT_TURNAUTO, AutoConstants.CONTINIUS_INPUT_TURNAUTO);
    pidController.setIntegratorRange(-AutoConstants.INTEGRAL_RANGE_TURNAUTO, AutoConstants.INTEGRAL_RANGE_TURNAUTO); 
    pidController.setSetpoint(ang);
    pidController.setTolerance(0.5);
  }

  @Override
  public void execute() {
    sensorAngle = driveTrain.getAngle();
    
    outputSpeed = MathUtil.clamp(pidController.calculate(sensorAngle, ang), -0.9, 0.9);
    driveTrain.tankDrive(outputSpeed, -outputSpeed);
    System.out.println("funcinando turn");
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