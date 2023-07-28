// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups.AutoCommands.Routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.autoCommands.*;
import frc.robot.commands.commandGroups.AutoCommands.PIDCommands.DrivePIDAuto;
import frc.robot.commands.commandGroups.AutoCommands.Routines.Autos.AutoMode;
import frc.robot.commands.commandGroups.AutoCommands.Routines.Autos.ShooterHeight;
import frc.robot.commands.commandGroups.AutoCommands.TimedCommands.DriveTimedAuto;
import frc.robot.commands.commandGroups.AutoCommands.TimedCommands.ShooterTimedAuto;
import frc.robot.commands.teleOpCommands.Angulation.AngulationEncoder;
import frc.robot.commands.teleOpCommands.Take.ShooterHigh;
import frc.robot.commands.teleOpCommands.Take.ShooterLow;
import frc.robot.commands.teleOpCommands.Take.ShooterMid;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.Take;

public class AutoRoutinesPID extends SequentialCommandGroup {
  
  public AutoRoutinesPID(ShooterHeight shooterHeight, AutoMode autoMode, DriveTrain driveTrain, Angulation angulation, Take intake) {
    Command shooterCommand = new ShooterHigh(intake);
    switch (shooterHeight) {
      case LOW:
        shooterCommand = new ShooterLow(intake);
        break;
      case MID:
        shooterCommand = new ShooterMid(intake);
        break;
      case HIGH:
        shooterCommand = new ShooterLow(intake);
        break;
    }

    SequentialCommandGroup commands = new SequentialCommandGroup(ChargeRoutine.BACKWARDS(driveTrain));
    switch (autoMode) {
      case MOB:
        commands = new SequentialCommandGroup(
          DrivePIDAuto.MOB(driveTrain)
        );
        break;

      case CHARGE:
        commands = new SequentialCommandGroup(
          ChargeRoutine.BACKWARDS(driveTrain)
        );
        break;

      case MOB_CHARGE:
        commands = new SequentialCommandGroup(
          DriveTimedAuto.MOB(driveTrain),
          ChargeRoutine.FORWARD(driveTrain)
        );
        break;

      case MOB_PIECE:
        commands = new SequentialCommandGroup(
          DrivePIDAuto.MOB(driveTrain),
          new ParallelCommandGroup(
            new TurnAutoPID(driveTrain, 180),
            new AngulationEncoder(angulation)
          ),
          new ShooterTimedAuto(intake, 0.5, 0.5, 1.5, true),
          new AngulationEncoder(angulation)
        );
        break;
    }

    addCommands(shooterCommand, commands);
  }

}
