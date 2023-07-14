// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.commandGroups.AutoCommands.TimedCommands.DriveTimedAuto;
import frc.robot.commands.commandGroups.AutoCommands.TimedCommands.ShooterTimedAuto;
import frc.robot.commands.commandGroups.Autos.AutoMode;
import frc.robot.commands.teleOpCommands.Drive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRoutinesTimed extends SequentialCommandGroup {

  /** Creates a new AutoRoutines. */
  public AutoRoutinesTimed(AutoMode selectedRoutine, DriveTrain driveTrain, Take intake) {
    switch (selectedRoutine) {
      case MID:
        addCommands(ShooterTimedAuto.MID(intake));
        break;

      case LOW:
        addCommands(ShooterTimedAuto.LOW(intake));
        break;

      case MID_MOB:
        addCommands(
          ShooterTimedAuto.MID(intake),
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
          ShooterTimedAuto.MID(intake),
          DriveTimedAuto.CHARGE(driveTrain)
        );
        break;

      case LOW_CHARGE:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DriveTimedAuto.CHARGE(driveTrain)
        );
        break;

      case MID_MOB_CHARGE:
        addCommands(
          ShooterTimedAuto.MID(intake),
          DriveTimedAuto.MOB_CHARGE(driveTrain)
        );
        break;

      case LOW_MOB_CHARGE:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DriveTimedAuto.MOB_CHARGE(driveTrain)
        );
        break;

      case MID_MOB_PIECE:
        addCommands(
          ShooterTimedAuto.MID(intake),
          DriveTimedAuto.MOB(driveTrain)
          //ANGUlAÇÃO A FAZER
        );
        break;

      case LOW_MOB_PIECE:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DriveTimedAuto.MOB(driveTrain)
          //ANGULACAO A FAZER
        );
        break;

      case MID_MOD_PIECE_CHARGE:
        addCommands(
          ShooterTimedAuto.MID(intake),
          DriveTimedAuto.MOB(driveTrain),
          //angulação a fazer
          DriveTimedAuto.CHARGE(driveTrain)
        );
        break;

      case LOW_MOD_PIECE_CHARGE:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DriveTimedAuto.MOB(driveTrain),
          //angulação a fazer
          DriveTimedAuto.CHARGE(driveTrain)
        );
        break;

      case MID_MOB_PIECE_MID:
        addCommands(
          ShooterTimedAuto.MID(intake),
          DriveTimedAuto.MOB(driveTrain),
          //Angulação a fazer
          DriveTimedAuto.GRID(driveTrain),
          ShooterTimedAuto.MID(intake)
        );
        break;

      case MID_MOB_PIECE_LOW:
        addCommands(
          ShooterTimedAuto.MID(intake),
          DriveTimedAuto.MOB(driveTrain),
          //Angulação a fazer
          DriveTimedAuto.GRID(driveTrain),
          ShooterTimedAuto.LOW(intake)
        );
        break;

      case LOW_MOB_PIECE_MID:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DriveTimedAuto.MOB(driveTrain),
          //Angulação a fazer
          DriveTimedAuto.GRID(driveTrain),
          ShooterTimedAuto.MID(intake)
        );
        break;

      case LOW_MOB_PIECE_LOW:
        addCommands(
          ShooterTimedAuto.LOW(intake),
          DriveTimedAuto.MOB(driveTrain),
          //Angulação a fazer
          DriveTimedAuto.GRID(driveTrain),
          ShooterTimedAuto.LOW(intake)
        );
        break;

      
    }
   
  }
}
