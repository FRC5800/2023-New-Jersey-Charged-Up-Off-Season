// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Routines;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
import frc.robot.Routines.Autos.AutoMode;
import frc.robot.Routines.Autos.ShooterHeight;
import frc.robot.commands.Angulation.Tele.AngulationEncoder2;
import frc.robot.commands.DriveTrain.Auto.DriveTimedAuto;
import frc.robot.commands.DriveTrain.Auto.TurnAutoPID;
import frc.robot.commands.DriveTrain.Auto.trajectory.FollowPath;
import frc.robot.commands.Take.Auto.ShooterTimedAuto;
import frc.robot.commands.Take.Tele.ShooterHigh;
import frc.robot.commands.Take.Tele.ShooterLow;
import frc.robot.commands.Take.Tele.ShooterMid;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.Take;

public class AutoRoutinesPID extends SequentialCommandGroup {

  public AutoRoutinesPID(Supplier<ShooterHeight> shooterHeight, Supplier<AutoMode> autoMode, DriveTrain driveTrain, Angulation angulation, Take intake) {
    switch (shooterHeight.get()) {
      case HIGH:
        addCommands(
          new ShooterHigh(intake)
        );
        break;
      default:
        addCommands(
          new ShooterLow(intake)
        );
        break;
    }
    //this(shooterHeight.get(), autoMode.get(), driveTrain, angulation, intake);
  }
  
  public AutoRoutinesPID(ShooterHeight shooterHeight, AutoMode autoMode, DriveTrain driveTrain, Angulation angulation, Take intake) {
    Command shooterCommand = new WaitCommand(0);
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
        break;
    }

    SequentialCommandGroup commands = new SequentialCommandGroup();
    switch (autoMode) {
      case MOB:
        commands = new SequentialCommandGroup(
          //DrivePIDAuto.MOB(driveTrain),
          new TurnAutoPID(driveTrain, 180),
          new FollowPath(driveTrain, Constants.TrajectoryConstants.xMobBig, Constants.TrajectoryConstants.yMobSmall)
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
          //DrivePIDAuto.MOB(driveTrain),
          new TurnAutoPID(driveTrain, 175),
          new FollowPath(driveTrain, Constants.TrajectoryConstants.xMobSmall, Constants.TrajectoryConstants.yMobSmall),
          //new ParallelCommandGroup(
            //new TurnAutoPID(driveTrain, 175),
          new AngulationEncoder2(angulation),
          //),
          new ShooterTimedAuto(intake, 0.5, 0.5, 1, true),
          new AngulationEncoder2(angulation),
          new TurnAutoPID(driveTrain, 178),
          new FollowPath(driveTrain, Constants.TrajectoryConstants.xMobSmall, Constants.TrajectoryConstants.yMobSmall),
          new ShooterMid(intake)
        );
        break;
      case NONE:
        break;
    }

    addCommands(shooterCommand, commands);
  }

}
