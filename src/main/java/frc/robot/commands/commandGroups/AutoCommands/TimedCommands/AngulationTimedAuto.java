// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups.AutoCommands.TimedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Angulation;

public class AngulationTimedAuto extends CommandBase {
  
  private final Angulation angulation;
  private final double speed = 1;
  private double time;
  private Timer timer = new Timer();
  private boolean down;

  public final static AngulationTimedAuto DOWN(Angulation angulation) {
    return new AngulationTimedAuto(angulation, 3 , true);
  }

  public final static AngulationTimedAuto UP(Angulation angulation) {
    return new AngulationTimedAuto(angulation, 3 , false);
  }
  
  public AngulationTimedAuto(Angulation angulation, double time, boolean down) {

    this.down = down;
    this.angulation = angulation;
    this.time = time;

    addRequirements(angulation);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if(down){
      angulation.setElevatorAngleSpeed(speed);
    }else{
    angulation.setElevatorAngleSpeed(-speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    angulation.setElevatorAngleSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
