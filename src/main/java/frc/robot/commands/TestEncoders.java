package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTrain.Auto.DrivePIDAuto;
import frc.robot.commands.DriveTrain.Auto.trajectory.FollowPath;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;


public class TestEncoders extends SequentialCommandGroup {
  
  public TestEncoders(Angulation angulation, Take take, DriveTrain driveTrain) {
    addCommands(
    new FollowPath(driveTrain),
    new DrivePIDAuto(driveTrain, 1)
    //new AngulationEncoder2(angulation)
    
    );
  }
}
