// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.DrivetrainConstants;

public class DriveTrainPID extends PIDSubsystem {
  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(DrivetrainConstants.kLeftMasterID);
  private VictorSPX leftSlave = new VictorSPX(DrivetrainConstants.kLeftSlaveID);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(DrivetrainConstants.kRightMasterID);
  private VictorSPX rightSlave = new VictorSPX(DrivetrainConstants.kRightSlaveID);

  WPI_PigeonIMU gyro = new WPI_PigeonIMU(8);
  
  public DriveTrainPID() {
    super(new PIDController(0.4, 0, 0));

    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();
    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    rightMaster.setInverted(true);
    rightSlave.setInverted(true);

    leftMaster.setInverted(false);
    leftSlave.setInverted(false);

    rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, DrivetrainConstants.kTimeOutEncoder);
    leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, DrivetrainConstants.kTimeOutEncoder);
  
    getController().setTolerance(0.12);
    setSetpoint(2);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    rightMaster.setVoltage(output);
    leftMaster.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    return getAverageEncoderMeters();
  }

  public double getLeftEncoderTicks() {
    double position;
      position = leftMaster.getSelectedSensorPosition(0);
    return position;
  }

  public double getRightEncoderTicks() {
    double position;
      position = rightMaster.getSelectedSensorPosition(0);
    return position;
  }

  public double encoderTicksToMeters(double encoderticks) {
		return -(encoderticks / 4096) * 0.471238898;
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
}
