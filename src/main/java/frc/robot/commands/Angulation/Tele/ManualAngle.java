// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Angulation.Tele;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Angulation;

public class ManualAngle extends CommandBase {
  Angulation angulation;
  XboxController subsystemsController;
  double angle;
  double speed;

  public ManualAngle(Angulation angulation, XboxController subsystemsController) {
    this.angulation = angulation;
    this.subsystemsController = subsystemsController;
    addRequirements(angulation);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    angle = angulation.getEncoderRotations();
    speed = subsystemsController.getRawAxis(XboxController.Axis.kRightY.value);
    if (speed > 0) {
      angulation.setElevatorAngleSpeed(speed*0.58);
    } else {
      angulation.setElevatorAngleSpeed(speed*0.66);
    }

    SmartDashboard.putNumber("angle", angle);
     
    /* 
    if (angle < Angulation.UP_POSITION + 0.1 && speed < 0) {
      angulation.setElevatorAngleSpeed(0);
    }*/
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
