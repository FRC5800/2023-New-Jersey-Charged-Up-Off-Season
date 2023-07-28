package frc.robot.commands.teleOpCommands.Take;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Take;

public class ShooterLow extends CommandBase {
  /** Creates a new AutoShooter. */
  private final Take intake;
  private double lowerSpd;
  private double upperSpd;
  private double time;
  private Timer timer = new Timer();
  private boolean in;

  
  public ShooterLow(Take intake) {
    time = 1;
    this.intake = intake;

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
    intake.setLowerShooterPercentage(-0.4);
    intake.setUpperShooterPercentage(-0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setLowerShooterPercentage(0);
    intake.setUpperShooterPercentage(0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}

