// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Take.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Take;

public class ShooterTimedAuto extends CommandBase {
  /** Creates a new AutoShooter. */
  private final Take intake;
  private double lowerSpd;
  private double upperSpd;
  private double time;
  private Timer timer = new Timer();
  private boolean in;

  public final static ShooterTimedAuto MID(Take intake) {
    return new ShooterTimedAuto(intake, 1, 1, 1 , false);
  }

  public final static ShooterTimedAuto LOW(Take intake) {
    return new ShooterTimedAuto(intake, 0.3, 0.3, 1 , false);
  }

  public final static ShooterTimedAuto IN(Take intake) {
    return new ShooterTimedAuto(intake, 0.65, 0.65, 1 , true);
  }
  
  public ShooterTimedAuto(Take intake, double lowerSpd, double upperSpd, double time, boolean in) {

    this.in = in;
    this.intake = intake;
    this.lowerSpd = lowerSpd;
    this.upperSpd = upperSpd;
    this.time = time;

    addRequirements(intake);
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
    setVoltage(upperSpd, lowerSpd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    setVoltage(0.0, 0.0);
  }

  public void setVoltage(double upper, double lower) {
    var dir = in ? 1 : -1;
    SmartDashboard.putNumber("Lower timed", lower);
    intake.setLowerShooterVoltage(lower*dir);
    intake.setUpperShooterPercentage(upper*dir);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
