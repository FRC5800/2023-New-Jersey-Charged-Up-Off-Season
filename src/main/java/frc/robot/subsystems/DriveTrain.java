// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  WPI_TalonSRX leftMaster = new WPI_TalonSRX(DriveConstants.kLeftMotorMasterPort);
  WPI_TalonSRX leftSlave = new WPI_TalonSRX(DriveConstants.kLeftMotorSlavePort);
  MotorControllerGroup leftGroup = new MotorControllerGroup(leftMaster, leftSlave);

  WPI_TalonSRX rightMaster = new WPI_TalonSRX(DriveConstants.kRightMotorMasterPort);
  WPI_TalonSRX rightSlave = new WPI_TalonSRX(DriveConstants.kRightMotorSlavePort);
  MotorControllerGroup rightGroup = new MotorControllerGroup(rightMaster, rightSlave);

  DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

  public ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

  public DriveTrain() {
    leftGroup.setInverted(true);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0 ,20);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20);

  }

  public void setVoltage(double voltage) {
    leftGroup.setVoltage(voltage);
    rightGroup.setVoltage(voltage);
  }

  public double getVoltage() {
    return ((leftGroup.get() + rightGroup.get()) / 2) * RobotController.getBatteryVoltage();
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftMaster.set(leftSpeed);
    rightMaster.set(rightSpeed);
  }

  //Conversions encoders
  public double encoderTicksToMeters(double encoderticks) {
		return (encoderticks / Constants.DriveConstants.kEncoderTicksPerRevolution) * Constants.DriveConstants.kWheelCircunference;
	}

  public double encoderMeterToTicks(double encoderMeters) {
		return (encoderMeters * Constants.DriveConstants.kEncoderTicksPerRevolution) / Constants.DriveConstants.kWheelCircunference;
	}

  //Reset encoders
  public void resetEncoders(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);

  }

  public void resetPigeon(){
    ahrs.reset();
 }

  //Get functions
  public double getLeftEncoderTicks() {
		double position;
			position = -leftMaster.getSelectedSensorPosition(0);
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
		return (getRightEncoderMeters() + getLeftEncoderMeters()) / 2.0;
	}

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}