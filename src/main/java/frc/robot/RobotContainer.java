// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Angulation.Tele.AngulationEncoder2;
import frc.robot.commands.Angulation.Tele.ManualAngle;
import frc.robot.commands.DriveTrain.Auto.trajectory.LowTakeHigh;
import frc.robot.commands.DriveTrain.Auto.trajectory.LowTakeMid;
import frc.robot.commands.DriveTrain.Auto.trajectoryNotUsing.EventGroupFollow;
import frc.robot.commands.DriveTrain.Tele.Drive;
import frc.robot.commands.Take.Tele.GetCube;
import frc.robot.commands.Take.Tele.GetCubeLimit;
import frc.robot.commands.Take.Tele.ShooterHigh;
import frc.robot.commands.Take.Tele.ShooterHigh2;
import frc.robot.commands.Take.Tele.ShooterLow;
import frc.robot.commands.Take.Tele.ShooterMid;
import frc.robot.commands.Take.Tele.ThrowMax;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Routines.ChargeRoutine;
import frc.robot.Routines.trajectory.TrajectoryRoutines;
import frc.robot.Routines.AutoRoutinesPID;
import frc.robot.Routines.Autos.AutoMode;
import frc.robot.Routines.Autos.ShooterHeight;
import frc.robot.subsystems.Angulation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Take;

public class RobotContainer {
  public DriveTrain driveTrain = new DriveTrain();
  public Angulation angulation = new Angulation();
  public Take take = new Take();

  public static XboxController driveController = new XboxController(OperatorConstants.kDriverControllerPort);
  public static XboxController subsystemsController = new XboxController(OperatorConstants.kSubsystemsControllerPort);

  SendableChooser<ShooterHeight> ShooterChooser = new SendableChooser<>();
  SendableChooser<AutoMode> AutoChooser = new SendableChooser<>();
  SendableChooser<Command> chooser = new SendableChooser<Command>();
  

  public RobotContainer() {
    ShooterChooser.setDefaultOption("HIGH", ShooterHeight.HIGH);
    ShooterChooser.addOption("MID", ShooterHeight.MID);
    ShooterChooser.addOption("LOW", ShooterHeight.LOW);
    ShooterChooser.addOption("NONE", ShooterHeight.NONE);

    AutoChooser.setDefaultOption("CHARGE", AutoMode.CHARGE);
    AutoChooser.addOption("MOB", AutoMode.MOB);
    AutoChooser.addOption("MOB_PIECE", AutoMode.MOB_PIECE);
    AutoChooser.addOption("MOB_CHARGE", AutoMode.MOB_CHARGE);
    AutoChooser.addOption("NONE", AutoMode.NONE);

    chooser.setDefaultOption("foward", ChargeRoutine.FORWARD(driveTrain));
    chooser.addOption("backwards", ChargeRoutine.BACKWARDS(driveTrain));

    SmartDashboard.putData(AutoChooser);
    SmartDashboard.putData(ShooterChooser);
    SmartDashboard.putData("chooser", chooser);
    

    driveTrain.setDefaultCommand(new Drive(driveTrain, driveController));
    angulation.setDefaultCommand(new ManualAngle(angulation, subsystemsController));
    take.setDefaultCommand(new GetCube(take, subsystemsController));
    
    configureBindings();
  }

  private void configureBindings() {

    
   // new JoystickButton(subsystemsController, XboxController.Button.kY.value).onTrue(new ShooterHigh(take));
    //new JoystickButton(subsystemsController, XboxController.Button.kX.value).onTrue(new ShooterMid(take));
    //new JoystickButton(subsystemsController, XboxController.Button.kA.value).onTrue(new ShooterTest(take, getCube));
    //new JoystickButton(subsystemsController, XboxController.Button.kA.value).onTrue(new ShooterLow(take));
    //new JoystickButton(subsystemsController, XboxController.Button.kB.value).onTrue(new ThrowMax(take));

    //new JoystickButton(driveController, XboxController.Button.kLeftBumper.value).onTrue(new GetTimed(take,4));
  
   // new JoystickButton(driveController, XboxController.Button.kA.value).onTrue(driveTrain.runOnce(() -> driveTrain.resetOdometry(new Pose2d(0,0,new Rotation2d()))));
   new JoystickButton(subsystemsController, XboxController.Button.kY.value).onTrue(new ShooterHigh(take));
   new JoystickButton(subsystemsController, XboxController.Button.kX.value).onTrue(new ShooterMid(take));
   new JoystickButton(subsystemsController, XboxController.Button.kA.value).onTrue(new ShooterLow(take));
    new JoystickButton(subsystemsController, XboxController.Button.kB.value).onTrue(new AngulationEncoder2(angulation));
    
    
    //new JoystickButton(subsystemsController, XboxController.Button.kRightBumper.value).whileTrue(new GetCube(take, subsystemsController));
    //new JoystickButton(subsystemsController, XboxController.Button.kLeftBumper.value).whileTrue(new ThrowCube(take, subsystemsController));
    //new JoystickButton(subsystemsController, XboxController.Button.kRightBumper.value).toggleOnTrue(new GetWithLimit(take));
    //new JoystickButton(subsystemsController, XboxController.Axis.kRightTrigger.value).toggleOnTrue(new GetInFloor(take, angulation));
  }

  public Command getAutonomousCommand() {
    //return new AutoRoutinesPID(ShooterChooser.getSelected(), AutoChooser.getSelected(), driveTrain, angulation, take);
//
    //return new AutoRoutinesPID(ShooterChooser::getSelected, AutoChooser::getSelected, driveTrain, angulation, take);
    return new AutoRoutinesPID(ShooterHeight.HIGH, AutoMode.CHARGE, driveTrain, angulation, take);

    //return chooser.getSelected();

    //return new TrajectoryRoutines(driveTrain, take, angulation);
    //return new LowTakeMid(driveTrain, take, angulation);
    //return new LowTakeHigh(driveTrain, take, angulation);

    //return new ChargeRoutine(driveTrain, true);
  }


}
