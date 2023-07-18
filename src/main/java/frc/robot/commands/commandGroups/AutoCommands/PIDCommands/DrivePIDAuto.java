// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups.AutoCommands.PIDCommands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DrivePIDAuto extends CommandBase {
  /** Creates a new DriveTime. */
  private final DriveTrain driveTrain;
  private double motorSpeed;
  private double distance;
  private double initialEncoder;
  private PIDController pid = new PIDController(Constants.AutoConstants.KP_DRIVEAUTO, Constants.AutoConstants.KI_DRIVEAUTO, Constants.AutoConstants.KD_DRIVEAUTO);

  public final static DrivePIDAuto MOB(DriveTrain driveTrain) {
    return new DrivePIDAuto(driveTrain, -4);
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
    pid.setTolerance(0.05, 1.9181);
    pid.setIntegratorRange(0.08, 0.5);
    driveTrain.resetEncoders();
    pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Erro setpoint", pid.getPositionError());
    SmartDashboard.putNumber("setpoint", pid.getPositionError());
    //SmartDashboard.putNumber("left encoder", driveTrain.getLeftEncoderMeters());
    //SmartDashboard.putNumber("right encoder", driveTrain.getRightEncoderMeters());
    driveTrain.tankDrive(pid.calculate(driveTrain.getLeftEncoderMeters(), distance), pid.calculate(driveTrain.getRightEncoderMeters(), distance));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
