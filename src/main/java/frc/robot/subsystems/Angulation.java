// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Angulation extends SubsystemBase {
  /** Creates a new Angulation. */

  CANSparkMax leftMaster = new CANSparkMax(DriveConstants.KLeftMotorMasterAngle, MotorType.kBrushed);
  CANSparkMax leftSlave = new CANSparkMax(DriveConstants.KLeftMotorSlaveAngle, MotorType.kBrushed);
  MotorControllerGroup leftGroup = new MotorControllerGroup(leftMaster, leftSlave);

  CANSparkMax rightMaster = new CANSparkMax(DriveConstants.KRightMotorMasterAngle, MotorType.kBrushed);
  WPI_VictorSPX rightSlave = new WPI_VictorSPX(DriveConstants.KRightMotorSlaveAngle);
  MotorControllerGroup rightgroup = new MotorControllerGroup(rightMaster, rightSlave);

  DifferentialDrive drive = new DifferentialDrive(leftGroup, rightMaster);

  RelativeEncoder rightEncoder = rightMaster.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);

  public Angulation() {
    rightgroup.setInverted(true);

    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

  }

  public void setElevatorAngleSpeed(double vel){
    rightMaster.set(vel);
    leftMaster.set(vel);
}
public double getEncoderAngle(){
  return (rightEncoder.getPosition())*360;
}
}
