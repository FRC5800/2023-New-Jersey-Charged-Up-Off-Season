// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Take extends SubsystemBase {
  /** Creates a new Take. */

  WPI_TalonSRX intakeUpper = new WPI_TalonSRX(ShooterConstants.KIntakeUpperID);
  WPI_TalonSRX intakeLower = new WPI_TalonSRX(ShooterConstants.KIntakeLowerID);

  private DigitalInput bufferSwitch = new DigitalInput(0);

  public Take() {
    intakeUpper.configFactoryDefault();
    intakeLower.configFactoryDefault();

    intakeUpper.setNeutralMode(NeutralMode.Brake);
    intakeLower.setNeutralMode(NeutralMode.Brake);

    intakeUpper.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
    intakeLower.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);

    intakeLower.setSensorPhase(true);

    intakeUpper.setInverted(false);
    intakeLower.setInverted(false);

    intakeLower.configNominalOutputForward(0, 20);
    intakeLower.configNominalOutputReverse(0, 20);
    intakeLower.configPeakOutputForward(1, 20);
    intakeLower.configPeakOutputReverse(-1, 20);

    intakeUpper.configNominalOutputForward(0, 20);
    intakeUpper.configNominalOutputReverse(0, 20);
    intakeUpper.configPeakOutputForward(1, 20);
    intakeUpper.configPeakOutputReverse(-1, 20);

    intakeLower.config_kF(0, 3.8, 20);
    intakeLower.config_kP(0, 0.000351, 20);
    intakeLower.config_kI(0, 0, 20);
    intakeLower.config_kD(0, 3, 20);

    intakeUpper.config_kF(0, 7.1, 20);
    intakeUpper.config_kP(0, 0.003, 20);
    intakeUpper.config_kI(0, 0, 20);
    intakeUpper.config_kD(0, 3, 20);
  
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("FimDeCurso", this.getEndOfRoad());
  }

  public void setUpperShooterPercentage(double vel){
    intakeUpper.set(ControlMode.PercentOutput, vel);
  }
  public void setLowerShooterPercentage(double vel){
    intakeLower.set(ControlMode.PercentOutput, vel); 
  }

  public void setUpperShooterVelocity(double vel){
    intakeUpper.set(ControlMode.Velocity, vel);
  }
  public void setLowerShooterVelocity(double vel){
    intakeLower.set(ControlMode.Velocity, vel);
  }

  public double getLowerEncoderRPM(){
    return (intakeUpper.getSelectedSensorVelocity()/4096)*60;
  }

  public double getUpperEncoderRPM(){
    return (intakeLower.getSelectedSensorVelocity()/4096)*60;
  }

  public double getLowerEncoderLinearVelocity(){
    return intakeLower.getSelectedSensorVelocity()/4096 * Math.PI * 2 * 5.08;
  }
  public double getUpperEncoderLinearVelocity(){
    return intakeUpper.getSelectedSensorVelocity()/4096 * Math.PI * 2 * 2.54;
  }

  public double getAverageEncoderRPM(){
    return (getLowerEncoderRPM() + getUpperEncoderRPM()) / 2;
  }

  public double getAverageEncoderLinearVelocity(){
    return (getLowerEncoderLinearVelocity() + getUpperEncoderLinearVelocity()) / 2 ;
  }

  public boolean getEndOfRoad() {
    return this.bufferSwitch.get();
  }
}
