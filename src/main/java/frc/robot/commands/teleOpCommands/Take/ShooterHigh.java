// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleOpCommands.Take;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.commandGroups.AutoCommands.TimedCommands.ShooterTimedAuto;
import frc.robot.subsystems.Take;

public class ShooterHigh extends SequentialCommandGroup{

  public ShooterHigh(Take take) {
    addCommands(
      new ShooterTimedAuto(take, 0, 0.48, 0.3 , true),
      new ShooterTimedAuto(take, 0.9, 0, 0.5, false),
      new ShooterTimedAuto(take, 0.7, 1, 0.6, false)
    );
  }

}
