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

  CANSparkMax rightMaster = new CANSparkMax(AngulationConstants.kAngleMasterID, MotorType.kBrushed);
  WPI_VictorSPX rightSlave = new WPI_VictorSPX(AngulationConstants.kAngleSlave0ID);
  MotorControllerGroup rightGroup = new MotorControllerGroup(rightMaster, rightSlave);

  CANSparkMax leftMaster = new CANSparkMax(AngulationConstants.kAngleSlave1ID, MotorType.kBrushed);
  CANSparkMax leftSlave = new CANSparkMax(AngulationConstants.kAngleSlave2ID, MotorType.kBrushed); 
  MotorControllerGroup leftGroup = new MotorControllerGroup(leftMaster, leftSlave);

  RelativeEncoder rightEncoder = rightMaster.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
  public static final double DOWN_POSITION = 0.6894;
  public static final double UP_POSITION = 0.2924;  //0.68505859375;

  public Angulation() {
    rightMaster.setInverted(false);
    rightSlave.setInverted(false);

    leftSlave.follow(leftMaster);

    leftMaster.setIdleMode(IdleMode.kBrake);
    leftSlave.setIdleMode(IdleMode.kBrake);
    rightMaster.setIdleMode(IdleMode.kBrake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

  }

  public void setElevatorAngleSpeed(double vel) {
    double speed = vel;
    rightGroup.set(vel);
    leftGroup.set(vel);
}
public double getEncoderRotations() {
  SmartDashboard.putNumber("Encoder Angulação", rightEncoder.getPosition());
  var position = rightEncoder.getPosition();
  //  return 550 - (position < 400 ? position+550 : position);
  if (position < 400){
    return position;
  }else{
    return 550-position;
  }
}
}
