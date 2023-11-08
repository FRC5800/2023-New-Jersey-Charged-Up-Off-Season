// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Angulation.Tele;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Angulation;

public class AngulationEncoder2 extends CommandBase {
 
  private final Angulation angulation;
  private double initialRotation;
  private boolean down;
  private double diff;
  private double diffUp;
  private double speed;
  private double speedUp;
  private double kUp;
  private double k;

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

    SmartDashboard.putBoolean("down", down);

    //diff = (Angulation.DOWN_POSITION - rotation) * (Angulation.DOWN_POSITION - rotation) /1000;

    diff = ((Angulation.DOWN_POSITION-15) - rotation);

    k = 0.01;
    speed = diff * k;
    speed = Math.min(0.3,speed);

    diffUp = rotation - (Angulation.UP_POSITION+12) ;

    kUp = 0.05;

    //if (angulation.getEncoderRotations() < 40) {kUp = kUp2;}

    speedUp = diffUp * kUp;
  
    speedUp = Math.min(0.45,speedUp);

    if(down){
      angulation.setElevatorAngleSpeed(speed);
      
      }else{
       angulation.setElevatorAngleSpeed(-speedUp); //0.61

    }

    SmartDashboard.putNumber("speed angulation", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angulation.setElevatorAngleSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    SmartDashboard.putBoolean("is finished", angulation.getEncoderRotations() >= Angulation.DOWN_POSITION-5);

    if (down){
      return angulation.getEncoderRotations() >= Angulation.DOWN_POSITION-20;
      
    }else{
      return angulation.getEncoderRotations() <= Angulation.UP_POSITION+8;
    }
  }
}
