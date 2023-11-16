// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public final class Constants {
  public static class DrivetrainConstants {
    public static final int kLeftMasterID = 1;
    public static final int kLeftSlaveID = 2;
    public static final int kRightMasterID = 3;
    public static final int kRightSlaveID = 4;
    public static final int kTimeOutEncoder = 20;
    public static final double kSPeedFast = 1 ;
    //public static final double kSPeedSlow = 0.6 ;
    public static final double kSPeedSlow = 0.7 ;

  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSubsystemsControllerPort = 1;

    public static JoystickButton buttonX = new JoystickButton(RobotContainer.subsystemsController, XboxController.Button.kX.value);
    
  }

  public static class DriveConstants {
    public static final int kLeftMotorMasterPort = 1;
    public static final int kLeftMotorSlavePort = 2;
    public static final int kRightMotorMasterPort = 3;
    public static final int kRightMotorSlavePort = 4;
    public static final float DEAD_ZONE = 0.1f;

    public static final int kEncoderTicksPerRevolution= 4096;
    public static final double kWheelCircunference = 0.471238898;

    public static final double KPTurnAuto = 3; //a definir
    public static final double KITurnAuto = 3; //a definir
    public static final double KDTurnAuto = 3; //a definir
    public static final double IrangeTurnAuto = 0.5; //a definir
    public static final double ContinuousInputTurnAuto = 180; //a definir

    public static final double robotMeasure = 0.94; //metros

  }
  public static class ShooterConstants{
    public static final int KIntakeUpperID = 6;
    public static final int KIntakeLowerID = 5;

    
    public static final double intakeKs = 0.81394;
    public static final double intakeKv = 0.87401;
    public static final double intakeKa = 0.10294;
    public static final double intakeKp = 0.27363;
  }
  public static class AngulationConstants {
    public static final int kAngleMasterID = 7;
  
    public static final double KMaxAngle = 300;
    public static final double KMinAngle = 0;
    public static final double KAngleMultiplier = 0.7;

  }

  public static class BalanceConstants {
    public static final double halfChargeStation = (6 + (0.5 / 12)) / 2;
    public static final double halfRobotLength = (29.5275591 / 2) / 12;
  }

  public static class AutoConstants {
    public static final double KP_TURNAUTO = 0.03;
    public static final double KI_TURNAUTO = 0.3;
    public static final double KD_TURNAUTO = 0.005;
    public static final double INTEGRAL_RANGE_TURNAUTO = 0.5;
    public static final double CONTINIUS_INPUT_TURNAUTO = 180;

    public static final double KP_DRIVEAUTO = 1.1;
    public static final double KI_DRIVEAUTO = 0.1; //0.01
    public static final double KD_DRIVEAUTO = 0.08; //0.009
  }

  public static class TrajectoryConstants {
    public static final double ksVolts = 0.70277;
    public static final double kvVoltSecondsPerMeter = 3.1943;
    public static final double kaVoltSecondsSquaredPerMeter = 0.68226;
    public static final double kPDriveVel = 0.32121;

    public static final double kTrackwidthMetersWithBumper = 0.92;
    public static final double kTrackwidthMeters = 0.62;

    public static final double kMaxSpeedMetersPerSecond = 2.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 8;
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double xMobSmall = 3.10;
    public static final double yMobSmall = 0;
    public static final double xMobBig = 4.30;
    public static final double yMobBig = 0;

  }
}
