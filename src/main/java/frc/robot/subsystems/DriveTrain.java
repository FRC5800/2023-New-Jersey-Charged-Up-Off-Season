// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

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
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public class DriveTrain extends SubsystemBase {

  private CANSparkMax leftMaster = new CANSparkMax(DrivetrainConstants.kLeftMasterID, MotorType.kBrushed);
  private WPI_VictorSPX leftSlave = new WPI_VictorSPX(DrivetrainConstants.kLeftSlaveID);
  private CANSparkMax rightMaster = new CANSparkMax(DrivetrainConstants.kRightMasterID, MotorType.kBrushed);
  private WPI_VictorSPX rightSlave = new WPI_VictorSPX(DrivetrainConstants.kRightSlaveID);

  private MotorControllerGroup rightMotors = new MotorControllerGroup(leftMaster, leftSlave);
  private MotorControllerGroup leftMotors = new MotorControllerGroup(rightMaster, rightSlave);


  private DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);
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

  private RelativeEncoder rightEncoder = rightMaster.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
  private RelativeEncoder leftEncoder = leftMaster.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);

  public DriveTrain() {
    SmartDashboard.putData(field);
  

    rightMaster.setInverted(true);
    rightSlave.setInverted(false);

    leftMaster.setInverted(true);
    leftSlave.setInverted(true);
    
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
    leftMotors.setVoltage(voltage);
    rightMotors.setVoltage(voltage);
  }

  
  public void kSpeedAlter() {
    kSpeedIsFast = !kSpeedIsFast;
    kSpeed = kSpeedIsFast ? kSpeedFast : kSpeedSlow;
  }

  //RESETTING SENSORS
  public void resetEncoders(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }
  public void resetGyro(){
    pigeon.reset();
  }

  //GET encoders
  public double getLeftEncoderTicks() {
    double position;
      position = leftEncoder.getPosition();
    return position;
  }
  public double getRightEncoderTicks() {
    double position;
      position = rightEncoder.getPosition();
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

    /**
     * Get RPM from left encoder encoder and convert it to linear velocity
   *
   * @return linear velocity of the left wheels
   */
  public double getLeftEncoderSpeed() {
    return leftEncoder.getVelocity() / 60 * Constants.DriveConstants.kWheelCircunference;
  }
    /**
     * Get RPM from right encoder encoder and convert it to linear velocity
   *
   * @return linear velocity of the right wheels
   */
  public double getRightEncoderSpeed() {
    return rightEncoder.getVelocity() / 60 * Constants.DriveConstants.kWheelCircunference;
  }

  public double getRightEncoderSpeedMeters() {
    return getRightEncoderSpeed() ;
  }
  public double getLeftEncoderSpeedMeters() {
    return getLeftEncoderSpeed() ;
  }


  //SET MASTER PERCENT OUTPUT
  /**
     * nao faz nada pois spark n tem essa função
   */
  public void setLeftMasterPercent(double percent){
    //nao faz nada pois spark n tem essa função
  }
  /**
     * nao faz nada pois spark n tem essa função
   */
  public void setRightMasterPercent(double percent){
    //rightMaster.set(ControlMode.PercentOutput, percent);
    //nao faz nada pois spark n tem essa função
  }

  public void setRightTargetPosition(double position){
    rightEncoder.setPosition(position);
  }
  public void setLeftTargetPosition(double position){
    leftEncoder.setPosition(position);
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
    leftMotors.setVoltage(leftVolts);
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

    SmartDashboard.putNumber("leftEncoder teleop", getLeftEncoderMeters());
    SmartDashboard.putNumber("rightEncoder teleop", getRightEncoderMeters());
    SmartDashboard.putNumber("rightEncoder speed", getRightEncoderSpeed());

    field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
  }
}
