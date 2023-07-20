// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandGroups.AutoCommands.Charge.ChargePigeon;
import frc.robot.commands.commandGroups.AutoCommands.Charge.ChargePigeonPID;
import frc.robot.commands.commandGroups.AutoCommands.Charge.KeepCharge;
import frc.robot.commands.commandGroups.AutoCommands.Charge.KeepCharge2;
import frc.robot.commands.teleOpCommands.Drive;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeRoutine extends SequentialCommandGroup {
  /** Creates a new ChargeStation. */
  public ChargeRoutine(DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ChargePigeon(driveTrain, 15, -0.83, 1.7),
      new ChargePigeon(driveTrain, 3, -0.579, 3),
      new KeepCharge2(driveTrain)
    );
  }
}
