// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain.Tele;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("speed", driveTrain.getSpeed());
    //SmartDashboard.putNumber("voltage", driveTrain.getVoltage());
    SmartDashboard.putNumber("battery Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("pitch", driveTrain.getPitch());
    SmartDashboard.putNumber("Yaw", driveTrain.getYaw());
    SmartDashboard.putNumber("Roll", driveTrain.getRoll());

    SmartDashboard.putNumber("speed", driveTrain.getAverageEncoderSpeed());

    driveTrain.drive(driveController);
      
    if (driveController.getYButtonPressed()){
      driveTrain.kSpeedAlter();
    }

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