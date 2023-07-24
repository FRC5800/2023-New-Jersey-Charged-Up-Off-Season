// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups.AutoCommands.Routines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.autoCommands.*;
import frc.robot.commands.commandGroups.AutoCommands.PIDCommands.DrivePIDAuto;
import frc.robot.commands.commandGroups.AutoCommands.Routines.Autos.AutoMode;
import frc.robot.commands.commandGroups.AutoCommands.TimedCommands.ShooterTimedAuto;
import frc.robot.commands.teleOpCommands.AngulationEncoder;
import frc.robot.commands.teleOpCommands.ShooterHigh;
import frc.robot.commands.teleOpCommands.ShooterMid;
import frc.robot.commands.trajectory.FollowPath;
import frc.robot.commands.trajectory.FollowPathMeters;
import frc.robot.subsystems.Angulation;
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
          new ChargeRoutine(driveTrain)
        );
        break;

      case HIGH_CHARGE:
        addCommands(
          new ShooterHigh(intake),
          new ChargeRoutine(driveTrain)
        );
        break;

      case MID_MOB_CHARGE:
        addCommands(
          new ShooterMid(intake),
          DrivePIDAuto.MOB(driveTrain),
          new ChargeRoutine(driveTrain)
        );
        break;

      case LOW_MOB_CHARGE:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DrivePIDAuto.MOB_C(driveTrain),
          new ChargeRoutine(driveTrain)
        );
        break;

      case MID_MOB_PIECE:
      addCommands(
        new ShooterMid(intake),
        DrivePIDAuto.MOB(driveTrain),
        new ParallelCommandGroup(
          new TurnAutoPID(driveTrain, 180),
          new AngulationEncoder(angulation)
        ),
        new ShooterTimedAuto(intake, 0.5, 0.5, 1.5, true),
        new AngulationEncoder(angulation)
      );
        break;

      case LOW_MOB_PIECE:
      addCommands(
        ShooterTimedAuto.LOW(intake),
        DrivePIDAuto.MOB(driveTrain),
        new FollowPathMeters(driveTrain),
        new ParallelCommandGroup(
          new TurnAutoPID(driveTrain, 180),
          new AngulationEncoder(angulation)
        ),
        new ShooterTimedAuto(intake, 0.5, 0.5, 1.5, true),
        new AngulationEncoder(angulation)
      );
        break;

        case HIGH_MOB_PIECE:
        addCommands(
          new ShooterHigh(intake),
          DrivePIDAuto.MOB(driveTrain),
          new ParallelCommandGroup(
            new TurnAutoPID(driveTrain, 180),
            new AngulationEncoder(angulation)
          ),
          new ShooterTimedAuto(intake, 0.5, 0.5, 1.5, true),
          new AngulationEncoder(angulation)
        );
          break;

      case MID_MOB_PIECE_CHARGE:
        addCommands(
          new ShooterHigh(intake),
          DrivePIDAuto.MOB(driveTrain),
          new ParallelCommandGroup(
            new TurnAutoPID(driveTrain, 180),
            new AngulationEncoder(angulation)
          ),
          new ShooterTimedAuto(intake, 0.5, 0.5, 1.5, true),
          new AngulationEncoder(angulation),
          new ChargeRoutine(driveTrain)
        );
          break;

      case LOW_MOB_PIECE_CHARGE: // falta testar
      addCommands(
        ShooterTimedAuto.LOW(intake),
        DrivePIDAuto.MOB(driveTrain),
        new ParallelCommandGroup(
          new TurnAutoPID(driveTrain, 180),
          new AngulationEncoder(angulation)
        ),
        new ShooterTimedAuto(intake, 0.5, 0.5, 1.5, true),
        new AngulationEncoder(angulation),
        // precisa alinhar com a charge
        new ChargeRoutine(driveTrain)
      );
        break;

      case MID_MOB_PIECE_LOW: // falta testar
        addCommands(
          new ShooterMid(intake),
          DrivePIDAuto.MOB(driveTrain),
          new ParallelCommandGroup(
            new TurnAutoPID(driveTrain, 180),
            new AngulationEncoder(angulation)
          ),
          new ShooterTimedAuto(intake, 0.5, 0.5, 1.5, true),
          new AngulationEncoder(angulation),
          DrivePIDAuto.MOB(driveTrain),
          ShooterTimedAuto.LOW(intake)
        );
          break;

      case LOW_MOB_PIECE_MID: // falta testar
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DrivePIDAuto.MOB(driveTrain),
          new ParallelCommandGroup(
            new TurnAutoPID(driveTrain, 180),
            new AngulationEncoder(angulation)
          ),
          new ShooterTimedAuto(intake, 0.5, 0.5, 1.5, true),
          new AngulationEncoder(angulation),
          DrivePIDAuto.MOB(driveTrain),
          new ShooterMid(intake)
        );
          break; 
    }
  }
}
