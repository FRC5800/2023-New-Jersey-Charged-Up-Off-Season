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

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngulationConstants;

public class Angulation extends SubsystemBase {
  /** Creates a new Angulation. */

  CANSparkMax angulationMotor = new CANSparkMax(AngulationConstants.kAngleMasterID, MotorType.kBrushed);

  RelativeEncoder encoder = angulationMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
  /**
   * Posição na qual a angulação está para baixo
   */
  public static final double DOWN_POSITION = 75;
  /**
   * Posição na qual a angulação está para cima
   */
  public static final double UP_POSITION = 2;  

  public Angulation() {
    angulationMotor.setInverted(false);

    angulationMotor.setIdleMode(IdleMode.kBrake);

    encoder.setPosition(0);
  }

  public void setElevatorAngleSpeed(double vel) {
    angulationMotor.set(vel);
    
}
public double getEncoderRotations() {
  SmartDashboard.putNumber("Encoder Angulação", encoder.getPosition());
  var position = encoder.getPosition();
  //  return 550 - (position < 400 ? position+550 : position);
  if (position < 400){
    return position;
  }else{
    return 550-position;
  }
}

public void setEncoderAngulationPosition(double position) {
  encoder.setPosition(position);
}

@Override
public void periodic() {
  SmartDashboard.putNumber("Angulation encoder ", getEncoderRotations());
  
}

}
