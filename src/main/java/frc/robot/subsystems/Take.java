// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Take extends SubsystemBase {
  /** Creates a new Take. */

  CANSparkMax intakeUpper = new CANSparkMax(5, MotorType.kBrushed);
  CANSparkMax intakeLower = new CANSparkMax(6, MotorType.kBrushed);

  private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.ShooterConstants.intakeKs, Constants.ShooterConstants.intakeKv, Constants.ShooterConstants.intakeKa);
  private PIDController pid = new PIDController(Constants.ShooterConstants.intakeKp, 0, 0);

  private DigitalInput bufferSwitch = new DigitalInput(0);

  public Take() {

    intakeUpper.setInverted(true);
    intakeLower.setInverted(true);
  
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("FimDeCurso", this.getEndOfRoad());
  }

  public void feedLowerVelocity(double vel) {
    setLowerShooterVoltage(feedForward.calculate(vel));
  }
  
  public void feedUpperVelocity(double vel) {
    setUpperShooterVoltage( feedForward.calculate(vel));
  }
  // public void feedPIDLowerVelocity(double vel) {
  //   setLowerShooterVoltage(Math.sqrt(pid.calculate(getLowerEncoderLinearVelocity(), vel*0.7) + feedForward.calculate(vel))*2);
  // }

  /**
   * Positive values get in
   * Negative values put out
   */
  public void setUpperShooterPercentage(double vel){
    intakeUpper.set(vel);
  }
  /**
   * Positive values put out
   * Negative values get in
   */
  public void setLowerShooterPercentage(double vel){
    intakeLower.set(vel); 
  }

  public void setUpperShooterVoltage(double vel){
    intakeUpper.setVoltage(vel);
  }
  public void setLowerShooterVoltage(double vel){
    intakeLower.setVoltage(vel); 
  }

  // public double getLowerEncoderRPS(){
  //   return intakeLower.getSelectedSensorVelocity()/4096*10;
  // }
  
  // public double getUpperEncoderRPS(){
  //   return intakeUpper.getSelectedSensorVelocity()/4096*10;
  // }

  // public double getLowerEncoderRPM(){
  //   return (intakeLower.getSelectedSensorVelocity()/4096)*600;
  // }

  // public double getUpperEncoderRPM(){
  //   return (intakeUpper.getSelectedSensorVelocity()/4096)*600;
  // }

  // public double getLowerEncoderLinearVelocity(){
  //   return intakeLower.getSelectedSensorVelocity()/4096 * Math.PI * 2 * 0.0381*10;
  // }
  // public double getUpperEncoderLinearVelocity(){
  //   return intakeUpper.getSelectedSensorVelocity()/4096 * Math.PI * 2 * 0.0508 *10;
  // }

  // public double getAverageEncoderRPM(){
  //   return (getLowerEncoderRPM() + getUpperEncoderRPM()) / 2;
  // }

  // public double getAverageEncoderLinearVelocity(){
  //   return (getLowerEncoderLinearVelocity() + getUpperEncoderLinearVelocity()) / 2 ;
  // }

  public boolean getEndOfRoad() {
    return this.bufferSwitch.get();
  }
}
