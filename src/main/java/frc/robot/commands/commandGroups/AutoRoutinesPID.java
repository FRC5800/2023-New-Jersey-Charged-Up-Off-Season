// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.autoCommands.*;
import frc.robot.commands.commandGroups.AutoCommands.PIDCommands.DrivePIDAuto;
import frc.robot.commands.commandGroups.AutoCommands.TimedCommands.ShooterTimedAuto;
import frc.robot.commands.commandGroups.Autos.AutoMode;
import frc.robot.commands.teleOpCommands.ShooterHigh;
import frc.robot.commands.teleOpCommands.ShooterLow;
import frc.robot.commands.teleOpCommands.ShooterMid;
import frc.robot.commands.trajectory.FollowPath;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

public class AutoRoutinesPID extends SequentialCommandGroup {
  
  public AutoRoutinesPID(AutoMode selectedRoutine, DriveTrain driveTrain, Angulation angulation, Take intake) {

    switch (selectedRoutine) {

      case PATH:
        addCommands(
          new FollowPath(driveTrain)
        );
        break;

      case HIGH:
        addCommands(
          new ShooterHigh(intake)
        );
        break;

      case MID:
        addCommands(
          new ShooterMid(intake)
        );
        break;

      case LOW:
        addCommands(ShooterTimedAuto.LOW(intake));
        break;

      case MID_MOB:
        addCommands(
          new ShooterMid(intake),
          DrivePIDAuto.MOB(driveTrain)
        );
        break;

      case LOW_MOB:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          // Drive command
         DrivePIDAuto.MOB(driveTrain)
        );
        break;

      case MID_CHARGE:
        addCommands(
          new ShooterMid(intake),
          new ChargeRoutine(driveTrain)
        );
        break;

      case LOW_CHARGE:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          // Balance command
          new ChargeRoutine(driveTrain)
        );
        break;

      case HIGH_CHARGE:
        addCommands(
          new ShooterHigh(intake),
          // Balance command
          new ChargeRoutine(driveTrain)
        );
        break;

      case MID_MOB_CHARGE:
        addCommands(
          new ShooterMid(intake),
          // Drive command
          DrivePIDAuto.MOB(driveTrain),
          // Balance command
          new ChargeRoutine(driveTrain)
        );
        break;

      case LOW_MOB_CHARGE:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          // Drive command
          DrivePIDAuto.MOB_C(driveTrain),
          // Balance command
          new ChargeRoutine(driveTrain)
        );
        break;

      case MID_MOB_PIECE:
        addCommands(
          
        );
        break;

      case LOW_MOB_PIECE:
      addCommands(
        
      );
        break;

      case MID_MOD_PIECE_CHARGE:
        addCommands(
          
        );
        break;

      case LOW_MOD_PIECE_CHARGE:
        addCommands(
          
        );
        break;

      case MID_MOB_PIECE_LOW:
        addCommands(
          
        );
        break;

      case LOW_MOB_PIECE_MID:
        addCommands(
          
        );
        break;
      
    }

  }

  private Command ChargeRoutine(DriveTrain driveTrain) {
    return null;
  }
}
