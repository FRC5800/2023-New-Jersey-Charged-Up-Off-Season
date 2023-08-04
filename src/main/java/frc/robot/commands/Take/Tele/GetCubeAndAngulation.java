package frc.robot.commands.Take.Tele;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Angulation.Tele.AngulationEncoder2;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.Take;


public class GetCubeAndAngulation extends SequentialCommandGroup {
  
  public GetCubeAndAngulation(Angulation angulation, Take take) {
    addCommands(
      new GetWithLimit(take),
      new AngulationEncoder2(angulation)
    );
  }
}
