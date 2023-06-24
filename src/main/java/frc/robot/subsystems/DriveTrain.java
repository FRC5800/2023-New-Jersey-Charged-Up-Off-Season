// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  WPI_TalonSRX leftMaster = new WPI_TalonSRX(DriveConstants.kLeftMotorMasterPort);
  WPI_TalonSRX leftSlave = new WPI_TalonSRX(DriveConstants.kLeftMotorSlavePort);

  WPI_TalonSRX rightMaster = new WPI_TalonSRX(DriveConstants.kRightMotorMasterPort);
  WPI_TalonSRX rightSlave = new WPI_TalonSRX(DriveConstants.kRightMotorSlavePort);

  DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
  
  private PigeonIMU pigeon = new PigeonIMU(8);

  public DriveTrain() {
    rightMaster.setInverted(true);
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0 ,20);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20);
    
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public void setVoltage(double voltage) {
    leftMaster.setVoltage(voltage);
    rightMaster.setVoltage(voltage);
  }

  public double getVoltage() {
    return ((leftMaster.get() + rightMaster.get()) / 2) * RobotController.getBatteryVoltage();
  }

  public double getSpeed() {
    return ((leftMaster.get() + rightMaster.get()) / 2);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftMaster.set(leftSpeed);
    rightMaster.set(rightSpeed);
  }

  //Conversions encoders
  public double encoderTicksToMeters(double encoderticks) {
		return (encoderticks / DriveConstants.kEncoderTicksPerRevolution) * DriveConstants.kWheelCircunference;
	}

  public double encoderMeterToTicks(double encoderMeters) {
		return (encoderMeters * DriveConstants.kEncoderTicksPerRevolution) / DriveConstants.kWheelCircunference;
	}

  //Reset encoders
  public void resetEncoders(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);

  }

  //Get functions
  public double getLeftEncoderTicks() {
		double position;
			position = -leftMaster.getSelectedSensorPosition(0);
      SmartDashboard.putNumber("Left Encoder Ticks", position);
		return position;
	}

  public double getRightEncoderTicks() {
		double position;
			position = -rightMaster.getSelectedSensorPosition(0);
		  return position;
	}

  public double getLeftEncoderMeters() {
		double meters = encoderTicksToMeters(getLeftEncoderTicks());
		SmartDashboard.putNumber("Left Encoder Position", meters);
		return meters;
	}

	public double getRightEncoderMeters() {
		double meters = encoderTicksToMeters(getRightEncoderTicks());
		SmartDashboard.putNumber("Right Encoder Position", meters);
		return meters;
	}

  public double getAverageEncoderMeters() {
    double averageMeters = (2*getRightEncoderMeters() + getLeftEncoderMeters()) / 2.0; 
		SmartDashboard.putNumber("Average Encoder Meters", averageMeters);
    return averageMeters;
	}

  public double getPitch(){
    SmartDashboard.putNumber("Pigeon Pitch", pigeon.getPitch());
    return -pigeon.getPitch();
 }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}