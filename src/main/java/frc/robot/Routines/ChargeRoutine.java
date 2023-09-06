// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTrain.Auto.Charge.ChargePigeon;
import frc.robot.commands.DriveTrain.Auto.Charge.KeepChargeArena;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeRoutine extends SequentialCommandGroup {

  public static ChargeRoutine FORWARD(DriveTrain driveTrain) {
    return new ChargeRoutine(driveTrain, true);
  }

  public static ChargeRoutine BACKWARDS(DriveTrain driveTrain) {
    return new ChargeRoutine(driveTrain, false);
  }

  public ChargeRoutine(DriveTrain driveTrain, boolean forward) {
    var dir = forward ? 1 : -1;

    addCommands(
      new ChargePigeon(driveTrain, 15*-dir, 0.8*dir, 1.7),
      new ChargePigeon(driveTrain, 6*-dir, 0.55*dir, 4),
      new KeepChargeArena(driveTrain)
    );
  }
}
