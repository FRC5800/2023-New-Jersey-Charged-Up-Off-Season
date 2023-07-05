// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Take extends SubsystemBase {
  /** Creates a new Take. */

  WPI_TalonSRX topMotor = new WPI_TalonSRX(DriveConstants.KTopShooterMotor);
  WPI_TalonSRX bottomMotor = new WPI_TalonSRX(DriveConstants.KBottomShooterMotor);

  public Take() {
    topMotor.setInverted(true);

    topMotor.setNeutralMode(NeutralMode.Brake);
    bottomMotor.setNeutralMode(NeutralMode.Brake);

    topMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0 ,20);
    bottomMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 20);
  }

  public void setShooterSpeed(double vel){
    topMotor.set(ControlMode.PercentOutput, vel);
    bottomMotor.set(ControlMode.PercentOutput, vel);
}

  public double getTopEncoderVelocity(){
    return topMotor.getSelectedSensorVelocity();
  }

  public double getBottomEncoderVelocity(){
    return bottomMotor.getSelectedSensorVelocity();
  }

  public double getAverageEncoderVelocity(){
    return (topMotor.getSelectedSensorVelocity() + bottomMotor.getSelectedSensorVelocity()) / 2;
  }
}
