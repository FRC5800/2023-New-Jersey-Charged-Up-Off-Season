
package frc.robot.commands.Take.Tele;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Take.Auto.ShooterTimedAuto;
import frc.robot.subsystems.Take;

public class ThrowMax extends SequentialCommandGroup{

  public ThrowMax(Take take) {
    addCommands(
      new ShooterTimedAuto(take, 0, 0.6, 0.4 , true),
      new ShooterTimedAuto(take, 1, -0.4, 0.9, false),
      new ShooterTimedAuto(take, 1, 1, 0.6, false)
    );
  }

}

