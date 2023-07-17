// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DriveTrain extends SubsystemBase {

  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(DrivetrainConstants.kLeftMasterID);
  private VictorSPX leftSlave = new VictorSPX(DrivetrainConstants.kLeftSlaveID);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(DrivetrainConstants.kRightMasterID);
  private VictorSPX rightSlave = new VictorSPX(DrivetrainConstants.kRightSlaveID);

  private DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);

  Pigeon2 pigeon = new Pigeon2(0);

  WPI_PigeonIMU gyro = new WPI_PigeonIMU(8);

  public DriveTrain() {
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
  }

  public void drive(XboxController xboxController){
    double y = -xboxController.getLeftY() * 0.8;
    double x = -xboxController.getRightX() * 0.8;

    diffDrive.arcadeDrive(y, x);
  }

  public void tankDrive(double left, double right) {
    diffDrive.tankDrive(left, right);
  }

  public void setVoltage(double voltage) {
    leftMaster.setVoltage(voltage);
    rightMaster.setVoltage(voltage);
  }

  public void resetOdometry(Pose2d pose){
    this.resetEncoders();
  }


  //RESETTING SENSORS
  public void resetEncoders(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);

  }

  public void resetGyro(){
    gyro.reset();
  }

  public void calibrateGyro(){
    gyro.calibrate();
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

  //SET MASTER PERCENT OUTPUT
  public void setLeftMasterPercent(double percent){
    leftMaster.set(ControlMode.PercentOutput, percent);
  }

  public void setRightMasterPercent(double percent){
    rightMaster.set(ControlMode.PercentOutput, percent);
  }

  public void setRightTargetPosition(double position){
    rightMaster.setSelectedSensorPosition(position);
  }


  public void setLeftTargetPosition(double position){
    leftMaster.setSelectedSensorPosition(position);
  }


  public double getHeading(){
    return gyro.getAngle();
  }


  public double getPitch(){
    return pigeon.getPitch();
  }

  public Rotation2d getRotation2D(){
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

public double getAngle() {
    return this.getHeading();
}
  }
