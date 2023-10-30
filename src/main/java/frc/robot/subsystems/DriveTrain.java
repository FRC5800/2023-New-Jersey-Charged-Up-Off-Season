// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public class DriveTrain extends SubsystemBase {

  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(DrivetrainConstants.kLeftMasterID);
  private VictorSPX leftSlave = new VictorSPX(DrivetrainConstants.kLeftSlaveID);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(DrivetrainConstants.kRightMasterID);
  private VictorSPX rightSlave = new VictorSPX(DrivetrainConstants.kRightSlaveID);

  private DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);
  private final DifferentialDriveOdometry odometry;
  public final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Constants.TrajectoryConstants.kTrackwidthMeters);

  private WPI_PigeonIMU pigeon = new WPI_PigeonIMU(8);
  private double initialPigeon;

  public Field2d field = new Field2d();

  private double kSpeed = Constants.DrivetrainConstants.kSPeedFast;
  private boolean kSpeedIsFast = true;
  private double kSpeedFast = Constants.DrivetrainConstants.kSPeedFast;
  private double kSpeedSlow = Constants.DrivetrainConstants.kSPeedSlow;

  public final PIDConstants pidConstants = new PIDConstants(Constants.TrajectoryConstants.kPDriveVel, 0, 0);

  public DriveTrain() {
    SmartDashboard.putData(field);
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
    odometry = new DifferentialDriveOdometry(pigeon.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters(), new Pose2d(0, 0, new Rotation2d(0)) /*,new Pose2d(5.0, 13.5, new Rotation2d()) */);
  }

  public void drive(XboxController xboxController){
    double y = -xboxController.getLeftY();
    double x = -xboxController.getRightX();

    y = y*kSpeed;
    x = x*kSpeed;
    diffDrive.arcadeDrive(y, x);
    rightMaster.set(rightMaster.get()*0.96825);
  }

  public void tankDrive(double left, double right) {
    diffDrive.tankDrive(left, right);
  }

  public void setVoltage(double voltage) {
    leftMaster.setVoltage(voltage);
    rightMaster.setVoltage(voltage);
  }

  
  public void kSpeedAlter() {
    kSpeedIsFast = !kSpeedIsFast;
    kSpeed = kSpeedIsFast ? kSpeedFast : kSpeedSlow;
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
		return -(encoderticks / 4096) * Constants.DriveConstants.kWheelCircunference;
  }

  public double getLeftEncoderMeters() {
    double meters = encoderTicksToMeters(getLeftEncoderTicks());
    //SmartDashboard.putNumber("Left Encoder Position", meters);
    return meters;
  }
  public double getRightEncoderMeters() {
    double meters = encoderTicksToMeters(getRightEncoderTicks());
    //SmartDashboard.putNumber("Right Encoder Position", meters);
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

  public double getRightEncoderSpeedMeters() {
    return encoderTicksToMeters(rightMaster.getSelectedSensorVelocity() * 10) ;
  }
  public double getLeftEncoderSpeedMeters() {
    return encoderTicksToMeters(leftMaster.getSelectedSensorVelocity() * 10) ;
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
  public double getHeading(){
    return pigeon.getRotation2d().getDegrees();
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
    return new DifferentialDriveWheelSpeeds(getLeftEncoderSpeedMeters(), getRightEncoderSpeedMeters());
  }  
  public void resetOdometry(Pose2d pose) {
    //pose = new Pose2d(3, 6, new Rotation2d(3)); /*rotation pode ser definido como (radiano), (cos,sen)*/
    odometry.resetPosition(
      pigeon.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters(), pose);
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
    diffDrive.feed();
  }
  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d(); //return de -180 até 180
  }
  public double getTurnRate() {
    return pigeon.getRate();
  }

  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
          this.resetOdometry(traj.getInitialPose());
      }),
      new PPRamseteCommand(
          traj, 
          this::getPose, // Pose supplier
          new RamseteController(),
          new SimpleMotorFeedforward(Constants.TrajectoryConstants.ksVolts, 
          Constants.TrajectoryConstants.kvVoltSecondsPerMeter, 
          Constants.TrajectoryConstants.kaVoltSecondsSquaredPerMeter ),
          this.driveKinematics, // DifferentialDriveKinematics
          this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
          new PIDController(Constants.TrajectoryConstants.kPDriveVel, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(Constants.TrajectoryConstants.kPDriveVel, 0, 0), // Right controller (usually the same values as left controller)
          this::tankDriveVolts, // Voltage biconsumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          this // Requires this drive subsystem
      )
  );
} 


  @Override
  public void periodic() {
    odometry.update(pigeon.getRotation2d(), getLeftEncoderMeters(), getRightEncoderMeters());

    
    SmartDashboard.putNumber(("speedMeters"), getLeftEncoderSpeedMeters());
    SmartDashboard.putNumber("leftEncoder teleop", getLeftEncoderMeters());
    SmartDashboard.putNumber("rightEncoder teleop", getRightEncoderMeters());
    SmartDashboard.putNumber("rightEncoder speed", getRightEncoderSpeed());
    SmartDashboard.putNumber("leftEncoder speed", getLeftEncoderSpeed());
    SmartDashboard.putNumber("pose X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("pose Y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("rotation", pigeon.getRotation2d().getDegrees());
    field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
  }
}
