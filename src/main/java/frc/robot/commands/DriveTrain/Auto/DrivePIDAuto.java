// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain.Auto;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DrivePIDAuto extends CommandBase {
  /** Creates a new DriveTime. */
  private final DriveTrain driveTrain;
  private double distance;
  private double initialEncoderLeft;
  private double initialEncoderRight;
  
  private PIDController pid = new PIDController(Constants.AutoConstants.KP_DRIVEAUTO, Constants.AutoConstants.KI_DRIVEAUTO, Constants.AutoConstants.KD_DRIVEAUTO);

  public final static DrivePIDAuto MOB(DriveTrain driveTrain) {
    return new DrivePIDAuto(driveTrain, -4);
  }
  public final static DrivePIDAuto MOB_F(DriveTrain driveTrain) {
    return new DrivePIDAuto(driveTrain, 4.4);
  }
  public final static DrivePIDAuto MOB_C(DriveTrain driveTrain) {
    return new DrivePIDAuto(driveTrain, -5);
  }
  
  public final static DrivePIDAuto CHARGE(DriveTrain driveTrain) {
    return new DrivePIDAuto(driveTrain, -1.);
  }
  
  public final static DrivePIDAuto MOB_CHARGE(DriveTrain driveTrain) {
    return new DrivePIDAuto(driveTrain, 1);
  }

  public final static DrivePIDAuto MOB_GRID(DriveTrain driveTrain) {
    return new DrivePIDAuto(driveTrain, 4);
  }

  public DrivePIDAuto(DriveTrain driveTrain, double distance) {
    this.driveTrain = driveTrain;
    this.distance = distance;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(0.1, 1.9181);
    pid.setIntegratorRange(0.1, 1.45);
    pid.reset();
    initialEncoderLeft = driveTrain.getLeftEncoderMeters();
    initialEncoderRight = driveTrain.getRightEncoderMeters();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double encoderLeft = driveTrain.getLeftEncoderMeters() - initialEncoderLeft;
    double encoderRight = driveTrain.getRightEncoderMeters() - initialEncoderRight;


    SmartDashboard.putNumber("Erro setpoint", pid.getPositionError());
    SmartDashboard.putNumber("setpoint", pid.getPositionError());
    SmartDashboard.putNumber("left encoder", encoderLeft);
    SmartDashboard.putNumber("right encoder", encoderRight);

    driveTrain.tankDrive(pid.calculate(encoderLeft, distance), pid.calculate(encoderRight, distance) );
    //driveTrain.tankDrive(MathUtil.clamp(pid.calculate(encoderLeft, distance), -0.88, 0.88), MathUtil.clamp(pid.calculate(encoderRight, distance), -0.88, 0.88));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    SmartDashboard.putBoolean("At setpoint", pid.atSetpoint());
    return pid.atSetpoint();

  }
}
