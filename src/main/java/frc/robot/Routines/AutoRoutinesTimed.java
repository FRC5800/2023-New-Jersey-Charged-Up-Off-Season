// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Routines.Autos.AutoMode;
import frc.robot.Routines.Autos.ShooterHeight;
import frc.robot.commands.Angulation.Auto.AngulationTimedAuto;
import frc.robot.commands.DriveTrain.Auto.DriveTimedAuto;
import frc.robot.commands.DriveTrain.Auto.DriveTurnAuto;
import frc.robot.commands.Take.Auto.ShooterTimedAuto;
import frc.robot.commands.Take.Tele.ShooterHigh;
import frc.robot.commands.Take.Tele.ShooterLow;
import frc.robot.commands.Take.Tele.ShooterMid;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

public class AutoRoutinesTimed extends SequentialCommandGroup {

  public AutoRoutinesTimed(ShooterHeight shooterHeight, AutoMode autoMode, DriveTrain driveTrain, Angulation angulation, Take intake) {
    Command shooterCommand = new ShooterHigh(intake);
    switch (shooterHeight) {
      case LOW:
        shooterCommand = new ShooterLow(intake);
        break;
      case MID:
        shooterCommand = new ShooterMid(intake);
        break;
      case HIGH:
        shooterCommand = new ShooterHigh(intake);
        break;
      case NONE:
        shooterCommand = new WaitCommand(0);
        break;
    }

    SequentialCommandGroup commands = new SequentialCommandGroup(ChargeRoutine.BACKWARDS(driveTrain));
    switch (autoMode) {
      case MOB:
        commands = new SequentialCommandGroup(
          DriveTimedAuto.MOB(driveTrain)
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
          DriveTimedAuto.MOB(driveTrain),
          new ParallelCommandGroup(
            new DriveTurnAuto(driveTrain, 180),
            AngulationTimedAuto.DOWN(angulation)
          ),
          new ShooterTimedAuto(intake, 0.5, 0.5, 1.5, true),
          AngulationTimedAuto.UP(angulation)
        );
        break;
      case NONE:
        shooterCommand = new WaitCommand(0);
        break;
    }

    addCommands(shooterCommand, commands);
  }

}
