// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Take.Tele;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Take;

public class Lancamento extends CommandBase {
  private Take take;
  private double h;
  private double y = 60;
  private double x;
  private double Dx;
  private double v;
  private double sen146 = 0.99646917312;
  private double g = 9.80665;
  /** Creates a new Lancamento. */
  public Lancamento(Take take) {
    this.take = take;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(take);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x = 2; //pegar da limelight
    Dx = 1.2 * x;

    h = y*(1.2*x - Math.pow(1.2*x, 2)) / (4*x-4* Math.pow(x, 2)) ;

    v = Math.sqrt(h*2*g/Math.pow(sen146, 2));

    v = 9;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //take.setLowerShooterVelocity(25 * 4096 / 600);
    take.feedPIDLowerVelocity(v);
    SmartDashboard.putNumber("Shooter speed lower RPM", take.getLowerEncoderRPM());
    SmartDashboard.putNumber("Shooter speed lower Linear", take.getLowerEncoderLinearVelocity());
    SmartDashboard.putNumber("Shooter speed lower RPS", take.getLowerEncoderRPS());
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
