// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Routines.cubeRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Angulation.Tele.AngulationEncoder2;
import frc.robot.commands.Angulation.Tele.GetDown;
import frc.robot.commands.Angulation.Tele.GetUp;
import frc.robot.commands.Take.Tele.GetCube;
import frc.robot.commands.Take.Tele.GetCubeLimit;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.Take;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TakeCubeRoutine extends SequentialCommandGroup {
  /** Creates a new TakeCubeRoutine. */
  public TakeCubeRoutine(Angulation angulation, Take take) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AngulationEncoder2(angulation),
      new GetCubeLimit(take, 2),
      new AngulationEncoder2(angulation)
    );
  }
}
