// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOpCommands.Angulation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Angulation;

public class AngulationEncoder2 extends CommandBase {
 
  private final Angulation angulation;
  private double initialRotation;
  private boolean down;
  private double diff;
  private double speed;

  public AngulationEncoder2(Angulation angulation) {
    this.angulation = angulation;

    addRequirements(angulation);
    
  }

  @Override
  public void initialize() {
    initialRotation = angulation.getEncoderRotations();
    down = true;
    if (initialRotation >= (Angulation.UP_POSITION+Angulation.DOWN_POSITION)/2) down = false;
  }

  @Override
  public void execute() {
    double rotation = angulation.getEncoderRotations();
    diff = (Angulation.DOWN_POSITION - rotation) * (Angulation.DOWN_POSITION - rotation) *10;

    speed = diff * 1.2;
    if(speed>0.75) {speed=0.75;}
    if(down){
      angulation.setElevatorAngleSpeed(speed);
      
      }else{
       angulation.setElevatorAngleSpeed(-0.50);

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
      return angulation.getEncoderRotations() >= Angulation.DOWN_POSITION-0.098;
      
    }else{
      return angulation.getEncoderRotations() <= Angulation.UP_POSITION+0.07;
    }
  }
}
