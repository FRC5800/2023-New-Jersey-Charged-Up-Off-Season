// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOpCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Angulation;

public class Angle extends CommandBase {
  /** Creates a new Angle. */
  Angulation angulation;
  XboxController subsystemsController;
  double angle;
  double speed;

  public Angle(Angulation angulation, XboxController subsystemsController) {
    this.angulation = angulation;
    this.subsystemsController = subsystemsController;
    addRequirements(angulation);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SmartDashboard.putNumber("Angulation rotation", angulation.getEncoderRotations());
    angle = angulation.getEncoderRotations();
    speed = subsystemsController.getRawAxis(XboxController.Axis.kRightY.value);
    //speed = speed * (((AngulationConstants.KMaxAngle - angle) * AngulationConstants.KAngleMultiplier / AngulationConstants.KMaxAngle - AngulationConstants.KMinAngle)+0.1); // quando sem imput indo pra tras
    angulation.setElevatorAngleSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
