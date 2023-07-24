// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups.AutoCommands.Routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.commandGroups.AutoCommands.Routines.Autos.AutoMode;
import frc.robot.commands.commandGroups.AutoCommands.TimedCommands.AngulationTimedAuto;
import frc.robot.commands.commandGroups.AutoCommands.TimedCommands.DriveTimedAuto;
import frc.robot.commands.commandGroups.AutoCommands.TimedCommands.DriveTurnAuto;
import frc.robot.commands.commandGroups.AutoCommands.TimedCommands.ShooterTimedAuto;
import frc.robot.commands.teleOpCommands.ShooterHigh;
import frc.robot.commands.teleOpCommands.ShooterMid;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

public class AutoRoutinesTimed extends SequentialCommandGroup {

  public AutoRoutinesTimed(AutoMode selectedRoutine, DriveTrain driveTrain, Take intake, Angulation angulation) {
    switch (selectedRoutine) {
      case MID:
        addCommands(new ShooterMid(intake));
        break;

      case LOW:
        addCommands(ShooterTimedAuto.LOW(intake));
        break;

      case HIGH:
        addCommands(new ShooterHigh(intake));
        break;

      case MID_MOB:
        addCommands(
          new ShooterMid(intake),
          DriveTimedAuto.MOB(driveTrain)
        );
        break;

      case LOW_MOB:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DriveTimedAuto.MOB(driveTrain)
        );
        break;

      case MID_CHARGE:
        addCommands(
          new ShooterMid(intake),
          DriveTimedAuto.CHARGE(driveTrain)
        );
        break;

      case LOW_CHARGE:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DriveTimedAuto.CHARGE(driveTrain)
        );
        break;

      case HIGH_CHARGE:
        addCommands(
          new ShooterHigh(intake),
          DriveTimedAuto.CHARGE(driveTrain)
        );
        break;

      case MID_MOB_CHARGE:
        addCommands(
          new ShooterMid(intake),
          DriveTimedAuto.MOB(driveTrain),
          DriveTimedAuto.MOB_CHARGE(driveTrain)
        );
        break;

      case LOW_MOB_CHARGE:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DriveTimedAuto.MOB(driveTrain),
          DriveTimedAuto.MOB_CHARGE(driveTrain)
        );
        break;

      case MID_MOB_PIECE:
        addCommands(
          new ShooterMid(intake),
          //DriveTimedAuto.MOB(driveTrain), 
          new DriveTurnAuto(driveTrain, 140),
          AngulationTimedAuto.DOWN(angulation),
          ShooterTimedAuto.IN(intake),
          //new WaitCommand(1),
          AngulationTimedAuto.UP(angulation)
        );
        break;

      case LOW_MOB_PIECE:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DriveTimedAuto.MOB(driveTrain), 
          new DriveTurnAuto(driveTrain, 180),
          AngulationTimedAuto.DOWN(angulation),
          ShooterTimedAuto.IN(intake),
          new WaitCommand(1),
          AngulationTimedAuto.UP(angulation)
        );
          break;
        
      case HIGH_MOB_PIECE:
        addCommands(
          new ShooterHigh(intake),
          DriveTimedAuto.MOB(driveTrain), 
          new DriveTurnAuto(driveTrain, 180),
          AngulationTimedAuto.DOWN(angulation),
          ShooterTimedAuto.IN(intake),
          new WaitCommand(1),
          AngulationTimedAuto.UP(angulation)
        );
            break;

      case MID_MOB_PIECE_CHARGE:
        addCommands(
          new ShooterMid(intake),
          DriveTimedAuto.MOB(driveTrain), 
          new DriveTurnAuto(driveTrain, 180),
          AngulationTimedAuto.DOWN(angulation),
          ShooterTimedAuto.IN(intake),
          new WaitCommand(1),
          AngulationTimedAuto.UP(angulation),
          DriveTimedAuto.MOB_CHARGE(driveTrain)
        );
        break;

      case LOW_MOB_PIECE_CHARGE:
        addCommands(
          new ShooterMid(intake),
          DriveTimedAuto.MOB(driveTrain), 
          new DriveTurnAuto(driveTrain, 180),
          AngulationTimedAuto.DOWN(angulation),
          ShooterTimedAuto.IN(intake),
          new WaitCommand(1),
          AngulationTimedAuto.UP(angulation),
          DriveTimedAuto.CHARGE(driveTrain)
        );
        break;

      case MID_MOB_PIECE_LOW:
        addCommands(
          new ShooterMid(intake),
          DriveTimedAuto.MOB(driveTrain), 
          new DriveTurnAuto(driveTrain, 180),
          AngulationTimedAuto.DOWN(angulation),
          ShooterTimedAuto.IN(intake),
          new WaitCommand(1),
          AngulationTimedAuto.UP(angulation),
          DriveTimedAuto.MOB(driveTrain),
          new DriveTurnAuto(driveTrain, 180),
          ShooterTimedAuto.LOW(intake)
        );
        break;

      case LOW_MOB_PIECE_MID:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DriveTimedAuto.MOB(driveTrain), 
          new DriveTurnAuto(driveTrain, 180),
          AngulationTimedAuto.DOWN(angulation),
          ShooterTimedAuto.IN(intake),
          new WaitCommand(1),
          AngulationTimedAuto.UP(angulation),
          DriveTimedAuto.MOB(driveTrain),
          new DriveTurnAuto(driveTrain, 180),
          new ShooterMid(intake)
        );
        break;
        
      default:
        break;
    }
  }
}
