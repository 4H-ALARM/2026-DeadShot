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
  public static final int hoodEncoderID = 9;
  public static final int neutralToggleButtonPort = 0;
  public static final int encoderResetButtonPort = 1;
  public static final String shooterCanbus = "endEffector";

  public static final double shooterkp = 10;
  public static final double shooterki = 0;
  public static final double shooterkd = 0;
  public static final double shooterks = 5;
  public static final double shooterkv = 0.053;
  public static final double shooterka = 0;
  public static final double shooterMaxAccel = 3600/60*40;
  public static final double shooterMaxSpeed = 3600/60;
  public static final double shooterJerk = 0.0;
  public static final double hoodToMotorRatio = 1.0;
  public static final double shooterRevUpTolerance = 5;
  public static final double shooterIndexerStartTolerance = 500;
  public static final double shooterFF = 0;

  public static final double indexerkp = 6;
  public static final double indexerki = 0.0;
  public static final double indexerkd = 0.0;
  public static final double indexerks = 0.08;
  public static final double indexerkv = 0.39;
  public static final double indexerka = 0.01;
  public static final double indexerCruiseVelocity = 6400/60;
  public static final double indexerAcceleration = 25*6400/60;
  public static final double indexerJerk = 0;

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
