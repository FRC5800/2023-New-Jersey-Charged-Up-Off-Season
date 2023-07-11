// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOpCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AngulationConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Angulation;

public class Angle extends CommandBase {
  /** Creates a new Angle. */
  Angulation angulation = new Angulation();
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
    angle = angulation.getEncoderAngle();
    speed = subsystemsController.getRawAxis(XboxController.Axis.kRightY.value);
    speed = speed * (((AngulationConstants.KMaxAngle - angle) * AngulationConstants.KAngleMultiplier / AngulationConstants.KMaxAngle - AngulationConstants.KMinAngle)+0.1); // quando sem imput indo pra tras
    angulation.setElevatorAngleSpeed(0.5);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
