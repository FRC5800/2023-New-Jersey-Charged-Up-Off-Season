// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOpCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.DriveTrain;

public class AngulationEncoderBom extends CommandBase {
  /** Creates a new DriveTime. */
  private final Angulation angulation;
  private double initialRotation;
  private boolean down;
  private boolean executando;

  public AngulationEncoderBom(Angulation angulation) {
    this.angulation = angulation;

    addRequirements(angulation);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialRotation = angulation.getEncoderRotations();
    SmartDashboard.putNumber("initialRotation", initialRotation);
    down = true;
    if (initialRotation >= (Angulation.UP_POSITION+Angulation.DOWN_POSITION)/2) down = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double diff = 0;
    double vel = 0;
    double rotation = angulation.getEncoderRotations();

    if(down){
      angulation.setElevatorAngleSpeed(0.65);
      
      }else{
       angulation.setElevatorAngleSpeed(-0.65);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angulation.setElevatorAngleSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (down){
      return angulation.getEncoderRotations() >= Angulation.DOWN_POSITION-0.11;
      
    }else{
      return angulation.getEncoderRotations() <= Angulation.UP_POSITION+0.11;
    }
  }
}
