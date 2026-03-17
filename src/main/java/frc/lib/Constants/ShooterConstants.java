// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Constants;

/** Shooter constants for the turret, the hood and the shooter itself */
public class ShooterConstants {

  // Device IDs
  public static final int topShooterMotorRightID = 24;
  public static final int topShooterMotorLeftID = 34;
  public static final int bottomShooterMotorRightID = 25;
  public static final int bottomShooterMotorLeftID = 35;
  public static final int hoodMotorID = 26;
  public static final int hoodMotorFollowerID = 36;
  public static final int indexerMotorID = 27;
  public static final int indexerMotorFollowerID = 37;
  public static final int hoodEncoderID = 4;
  public static final int neutralToggleButtonPort = 0;
  public static final int encoderResetButtonPort = 1;
  public static final String shooterCanbus = "endEffector";

  // Turret Tuning Constants
  public static final double turretkp = 1.0;
  public static final double turretki = 0.0;
  public static final double turretkd = 0.0;
  public static final double turretks = 0.0;
  public static final double turretkv = 0.0;
  public static final double turretka = 1.0;
  public static final double turretMaxAccel = 50.0;
  public static final double turretMaxSpeed = 50.0;
  public static final double turretJerk = 50.0;
  public static final double turretRatio = 40 / 90;
  public static final double maxRotationPositive = 36000000.0;
  public static final double maxRotationNegative = 36000000.0;
  public static final double turretTolerance = 0;
  public static final double turretPoseOffset = 0.3175;

  public static final double shooterkp = 3.25;
  public static final double shooterki = 0.25;
  public static final double shooterkd = 0.0;
  public static final double shooterks = 2;
  public static final double shooterkv = 0.39;
  public static final double shooterka = 0.33;
  public static final double shooterMaxAccel = 50.0;
  public static final double shooterMaxSpeed = 50.0;
  public static final double shooterJerk = 50.0;
  public static final double hoodToMotorRatio = 1.0;

  public static final double indexerkp = 1.0;
  public static final double indexerki = 0.0;
  public static final double indexerkd = 0.0;
  public static final double indexerks = 0.0;
  public static final double indexerkv = 0.0;
  public static final double indexerka = 0.0;

  public static final double hoodkp = 1.0;
  public static final double hoodki = 0.0;
  public static final double hoodkd = 0.0;
  public static final double hoodks = 0.0;
  public static final double hoodkv = 0.0;
  public static final double hoodka = 1.0;
  public static final double hoodkg = 1.0;
  public static final double hoodMaxAccel = 50.0;
  public static final double hoodMaxSpeed = 50.0;
  public static final double hoodJerk = 50.0;
  public static final double hoodGearRatio = 1;
}
