// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DriveTrain extends SubsystemBase {

  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(DrivetrainConstants.kLeftMasterID);
  private VictorSPX leftSlave = new VictorSPX(DrivetrainConstants.kLeftSlaveID);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(DrivetrainConstants.kRightMasterID);
  private VictorSPX rightSlave = new VictorSPX(DrivetrainConstants.kRightSlaveID);

  //private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave);
  //private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave);

  private DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);
  private final DifferentialDriveOdometry odometry;

  private WPI_PigeonIMU pigeon = new WPI_PigeonIMU(8);
  private double initialPigeon;
  

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
  
    initialPigeon = pigeon.getRoll();
    resetEncoders();
    odometry = new DifferentialDriveOdometry(pigeon.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters() /*,new Pose2d(5.0, 13.5, new Rotation2d()) */);
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

  //RESETTING SENSORS
  public void resetEncoders(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }
  public void resetGyro(){
    pigeon.reset();
  }

  //GET encoders
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
  public double getAverageEncoderSpeed() {
    return (getRightEncoderSpeed() + getLeftEncoderSpeed()) / 2.0;
  }

  public double getLeftEncoderSpeed() {
    return leftMaster.getSelectedSensorVelocity();
  }
  public double getRightEncoderSpeed() {
    return rightMaster.getSelectedSensorVelocity();
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


  //pigeon
  public double getAngle(){
    return pigeon.getAngle();
  }
  public double getPitch(){
    return pigeon.getPitch();//-initialPigeon;
  }
  public double getYaw(){
    return pigeon.getYaw();//-initialPigeon;
  }
  public double getRoll(){
    return pigeon.getRoll()-initialPigeon;//-initialPigeon;
  }

  //trajectory
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderSpeed(), getRightEncoderSpeed());
  }  
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    //pose = new Pose2d(3, 6, new Rotation2d(3)); /*rotation pode ser definido como (radiano), (cos,sen)*/
    odometry.resetPosition(
      pigeon.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters(), pose);
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
    diffDrive.feed();
  }
  public double getHeading() {
    return pigeon.getRotation2d().getDegrees(); //return de -180 até 180
  }
  public double getTurnRate() {
    return pigeon.getRate();
  }

  @Override
  public void periodic() {
    odometry.update(pigeon.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters());
  }

  @Override
  public void simulationPeriodic() {
  }
}
