// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.autoCommands.*;
import frc.robot.commands.commandGroups.Autos.AutoMode;

public class AutoRoutinesPID extends SequentialCommandGroup {
  
  

  public AutoRoutinesPID(AutoMode selectedRoutine, DriveTrain driveTrain) {
    /* 
    if(selectedRoutine == AutoMode.TESTES_CHARGESTATION){
      addCommands(
        new BalanceCommand1(driveTrain)
      );
    } 
    else if(selectedRoutine == AutoMode.PID_MOBILITY){
      addCommands(
        new DriveAuto(driveTrain, 4)
      );
    } 
    else if(selectedRoutine == AutoMode.PID_MID_MOB_PICK){
      addCommands(
        // Comando de jogar GP
        new DriveAuto(driveTrain, 5)
        // Comando de pegar GP
      );
    } 
    else if(selectedRoutine == AutoMode.PID_MID_MOB_PICK_MID){
      addCommands(
        // Comando de jogar GP
        new DriveAuto(driveTrain, 5)
        // Comando de pegar GP
        // Comando de jogar GP
      );
    } 
    else if(selectedRoutine == AutoMode.PID_MID_CHARGE){
      addCommands(
        // Comando de jogar GP
        new BalanceCommand1(driveTrain)
      );
    } 
    else if(selectedRoutine == AutoMode.PID_MID_MOB_PICK_CHARGE){
      addCommands(
        // Comando de jogar GP
        new DriveAuto(driveTrain, 5),
        // Comando de pegar GP
        new BalanceCommand1(driveTrain)
      );
    } 
    else if(selectedRoutine == AutoMode.PID_MID_MOB_PICK_MID_CHARGE){
      addCommands(
        // Comando de jogar GP
        new DriveAuto(driveTrain, 5),
        // Comando de pegar GP
        new BalanceCommand1(driveTrain)
        // Comando de jogar GP
      );
    }
    */
  }
}
