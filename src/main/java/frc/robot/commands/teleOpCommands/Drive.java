// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOpCommands;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  private final DriveTrain driveTrain;
  private final XboxController driveController;
  //private boolean toggleSPD = false;

  public Drive(DriveTrain driveTrain, XboxController drivController) {
    this.driveTrain = driveTrain;
    this.driveController = drivController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
  }

  private boolean between(double v, double n1, double n2){
    if (n1 < v && v < n2){
      return false;
    }
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("speed", driveTrain.getSpeed());
    //SmartDashboard.putNumber("voltage", driveTrain.getVoltage());
    SmartDashboard.putNumber("encoder", driveTrain.getLeftEncoderTicks());
    SmartDashboard.putNumber("battery Voltage", RobotController.getBatteryVoltage());
    double axisLeft = -driveController.getRawAxis(XboxController.Axis.kLeftY.value);
    double axisRight = -driveController.getRawAxis(XboxController.Axis.kRightX.value);
    double leftSpeed = 0;
    double rightSpeed = 0;


    //if ((axisLeft > DriveConstants.DEAD_ZONE || axisLeft < -DriveConstants.DEAD_ZONE)){
    if ((between(axisLeft, -DriveConstants.DEAD_ZONE,DriveConstants.DEAD_ZONE))){
      leftSpeed += -axisLeft * 0.95 * Math.cos(axisRight);
      rightSpeed += -axisLeft * 0.95 * Math.cos(axisRight);
    }     
  
  
    //if(axisRight > DriveConstants.DEAD_ZONE || axisRight < -DriveConstants.DEAD_ZONE){
    if(between(axisRight, -DriveConstants.DEAD_ZONE,DriveConstants.DEAD_ZONE)){
      leftSpeed += 0.95 * Math.sin(axisRight);
      rightSpeed += -0.95 * Math.sin(axisRight);
    }
    
    driveTrain.tankDrive((-Math.min(leftSpeed, 0.95)),(-Math.min(rightSpeed, 0.95)));

      
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