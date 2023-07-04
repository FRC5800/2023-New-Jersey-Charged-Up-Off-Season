// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.autoCommands.*;

public class AutoRoutines extends SequentialCommandGroup {
  
  public enum AutoMode{
    TESTES_CHARGESTATION,
    PID_MID_MOB_CHARGE,
    PID_MID_CHARGE,
    PID_MID_PICK_MID,
    PID_MID_PICK,
    PID_MOBILITY,
}

  public AutoRoutines(AutoMode selectedRoutine, DriveTrain driveTrain) {
  
    if(selectedRoutine == AutoMode.TESTES_CHARGESTATION){
      addCommands(
        new BalanceCommand1(driveTrain)
      );
    }
  }
}
