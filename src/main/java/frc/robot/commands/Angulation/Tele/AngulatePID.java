// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Angulation.Tele;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Angulation;

public class AngulatePID extends CommandBase {
  /** Creates a new AngulatePID. */
  private Angulation angulation;
  private double initialAngle;
  public AngulatePID() {
    this.angulation = angulation;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(angulation);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = angulation.getEncoderRotations();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
