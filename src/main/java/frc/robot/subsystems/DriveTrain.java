// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  WPI_VictorSPX leftMaster = new WPI_VictorSPX(DriveConstants.kLeftMotorMasterPort);
  WPI_VictorSPX leftSlave = new WPI_VictorSPX(DriveConstants.kLeftMotorSlavePort);

  WPI_VictorSPX rightMaster = new WPI_VictorSPX(DriveConstants.kRightMotorMasterPort);
  WPI_VictorSPX rightSlave = new WPI_VictorSPX(DriveConstants.kRightMotorSlavePort);

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  public WPI_Pigeon2 pigeon = new WPI_Pigeon2(8);

  public DriveTrain() {
    rightMaster.setInverted(true);
    rightSlave.setInverted(true);
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

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
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  //Conversions encoders
  public double encoderTicksToMeters(double encoderticks) {
		return (encoderticks / DriveConstants.kEncoderTicksPerRevolution) * DriveConstants.kWheelCircunference;
	}

  public double encoderMeterToTicks(double encoderMeters) {
		return (encoderMeters * DriveConstants.kEncoderTicksPerRevolution) / DriveConstants.kWheelCircunference;
	}

  //Resets
  public void resetEncoders(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);

  }

  public void resetGyro(){
    pigeon.reset();
  }

  //Get functions
  public double getAngle() {
    return pigeon.getAngle();
  }
  public double getHeading(){
    return pigeon.getYaw();
  }
  public double getInclination(){
    return pigeon.getPitch();
  }

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
		double meters = encoderMeterToTicks(getLeftEncoderTicks());
		SmartDashboard.putNumber("Left Encoder Position", meters);
		return meters;
	}

	public double getRightEncoderMeters() {
		double meters = encoderTicksToMeters(getRightEncoderTicks());
		SmartDashboard.putNumber("Right Encoder Position", meters);
		return meters;
	}

  public double getAverageEncoderMeters() {
    double averageMeters = (getRightEncoderMeters() + getLeftEncoderMeters()) / 2;
		SmartDashboard.putNumber("Average Encoder Meters", averageMeters);
    return averageMeters;
	}


  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}