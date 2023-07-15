// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups.AutoCommands.TimedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.Take;

public class AngulationTimedAuto extends CommandBase {
  /** Creates a new AutoShooter. */
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
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(down){
      angulation.setElevatorAngleSpeed(speed);
    }else{
    angulation.setElevatorAngleSpeed(-speed);
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
    return timer.get() > time;
  }
}
