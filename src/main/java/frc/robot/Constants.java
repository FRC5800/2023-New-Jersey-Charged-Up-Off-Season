// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSubsystemsControllerPort = 1;
    
  }

  public static class DriveConstants {
    public static final int kLeftMotorMasterPort = 3;
    public static final int kLeftMotorSlavePort = 4;
    public static final int kRightMotorMasterPort = 23;
    public static final int kRightMotorSlavePort = 2;

    //a definir
    public static final int KLeftMotorSlaveAngle = 1;
    public static final int KLeftMotorMasterAngle = 54;
    public static final int KRightMotorSlaveAngle = 55;
    public static final int KRightMotorMasterAngle = 13;
    public static final int KTopShooterMotor = 67;
    public static final int KBottomShooterMotor = 87;
    //a definir

    public static final double KMaxAngle = 0;
    public static final double KMinAngle = 0;
    public static final double KAngleMultiplier = 0.7;

    public static final float DEAD_ZONE = 0.1f;

    public static final int kEncoderTicksPerRevolution= 2048;
    public static final double kWheelCircunference = 0.471238898;

    //Calculando quantos metros para se equilibrar na charge(obs: tudo em feet)
    public static final double halfChargeStation = (6 + (0.5 / 12)) / 2;
    public static final double halfRobotLength = (29.5275591 / 2) / 12;
    
  }

  public static class AngulationConstants {

  }

  public static class BalanceConstants {
    public static final double halfChargeStation = (6 + (0.5 / 12)) / 2;
    public static final double halfRobotLength = (29.5275591 / 2) / 12;
  }

  public static class AutoConstants {
    public static final double KP_TURNAUTO = 0.03;
    public static final double KI_TURNAUTO = 0.2;
    public static final double KD_TURNAUTO = 0.005;
    public static final double INTEGRAL_RANGE_TURNAUTO = 0.5;
    public static final double CONTINIUS_INPUT_TURNAUTO = 180;

    public static final double KP_DRIVEAUTO = 0;
    public static final double KI_DRIVEAUTO = 0;
  }
}
