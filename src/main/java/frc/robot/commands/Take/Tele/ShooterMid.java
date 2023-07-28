// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Take.Tele;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Take.Auto.ShooterTimedAuto;
import frc.robot.subsystems.Take;

public class ShooterMid extends SequentialCommandGroup{

  public ShooterMid(Take take) {
    addCommands(
      new ShooterTimedAuto(take, 0, 0.48, 0.3 , true),
      new ShooterTimedAuto(take, 0.4, 0, 1 , false),
      new ShooterTimedAuto(take, 0.4, 1, 0.4, false)
    );
  }

}