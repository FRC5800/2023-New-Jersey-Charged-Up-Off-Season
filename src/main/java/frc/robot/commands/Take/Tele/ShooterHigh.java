// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Take.Tele;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Take.Auto.ShooterTimedAuto;
import frc.robot.subsystems.Take;

public class ShooterHigh extends SequentialCommandGroup{

  public ShooterHigh(Take take) {
    addCommands(
      new ShooterTimedAuto(take, 0.3, 0.4, 0.7 , false),
      //new ShooterTimedAuto(take, 0.45, -0.4, 0.9 , false),
      new ShooterTimedAuto(take, -0.5, 0.3, 0.8 , false),
      new ShooterTimedAuto(take, -0.5, -0.7, 0.4, false) //antes tensão tava 0.4
    );
  }

}
