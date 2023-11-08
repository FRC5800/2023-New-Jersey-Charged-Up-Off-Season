// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Routines.trajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Routines.ChargeRoutine;
import frc.robot.commands.DriveTrain.Auto.trajectory.LowTakeChargePath;
import frc.robot.commands.DriveTrain.Auto.trajectoryNotUsing.EventGroupFollow;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LowTakeChargeRoutine extends SequentialCommandGroup {
  /** Creates a new TrajectoryRoutines. */
  public LowTakeChargeRoutine(DriveTrain driveTrain, Take take, Angulation angulation) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LowTakeChargePath(driveTrain, take, angulation),
      new ChargeRoutine(driveTrain, true)
    );
  }
}
