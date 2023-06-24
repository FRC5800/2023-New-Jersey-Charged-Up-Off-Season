// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class AutoRoutines extends SequentialCommandGroup {
  
  public enum AutoMode{
    ChargeStation
}

  public AutoRoutines(AutoMode selectedRoutine, DriveTrain driveTrain) {
  
    if(selectedRoutine == AutoMode.ChargeStation){
      addCommands(
        new BalanceCommand1(driveTrain)
      );
    }
  }
}
